# Copyright (c) 2022 NVIDIA CORPORATION.  All rights reserved.
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

from __future__ import annotations

import ast
import builtins
import ctypes
import inspect
import math
import re
import sys
import textwrap
import types
from typing import Any, Callable, Mapping

import warp.config
from warp.types import *


class WarpCodegenError(RuntimeError):
    def __init__(self, message):
        super().__init__(message)


class WarpCodegenTypeError(TypeError):
    def __init__(self, message):
        super().__init__(message)


class WarpCodegenAttributeError(AttributeError):
    def __init__(self, message):
        super().__init__(message)


class WarpCodegenKeyError(KeyError):
    def __init__(self, message):
        super().__init__(message)


# map operator to function name
builtin_operators = {}

# see https://www.ics.uci.edu/~pattis/ICS-31/lectures/opexp.pdf for a
# nice overview of python operators

builtin_operators[ast.Add] = "add"
builtin_operators[ast.Sub] = "sub"
builtin_operators[ast.Mult] = "mul"
builtin_operators[ast.MatMult] = "mul"
builtin_operators[ast.Div] = "div"
builtin_operators[ast.FloorDiv] = "floordiv"
builtin_operators[ast.Pow] = "pow"
builtin_operators[ast.Mod] = "mod"
builtin_operators[ast.UAdd] = "pos"
builtin_operators[ast.USub] = "neg"
builtin_operators[ast.Not] = "unot"

builtin_operators[ast.Gt] = ">"
builtin_operators[ast.Lt] = "<"
builtin_operators[ast.GtE] = ">="
builtin_operators[ast.LtE] = "<="
builtin_operators[ast.Eq] = "=="
builtin_operators[ast.NotEq] = "!="

builtin_operators[ast.BitAnd] = "bit_and"
builtin_operators[ast.BitOr] = "bit_or"
builtin_operators[ast.BitXor] = "bit_xor"
builtin_operators[ast.Invert] = "invert"
builtin_operators[ast.LShift] = "lshift"
builtin_operators[ast.RShift] = "rshift"

comparison_chain_strings = [
    builtin_operators[ast.Gt],
    builtin_operators[ast.Lt],
    builtin_operators[ast.LtE],
    builtin_operators[ast.GtE],
    builtin_operators[ast.Eq],
    builtin_operators[ast.NotEq],
]


def op_str_is_chainable(op: str) -> builtins.bool:
    return op in comparison_chain_strings


def get_annotations(obj: Any) -> Mapping[str, Any]:
    """Alternative to `inspect.get_annotations()` for Python 3.9 and older."""
    # See https://docs.python.org/3/howto/annotations.html#accessing-the-annotations-dict-of-an-object-in-python-3-9-and-older
    if isinstance(obj, type):
        return obj.__dict__.get("__annotations__", {})

    return getattr(obj, "__annotations__", {})


def struct_instance_repr_recursive(inst: StructInstance, depth: int) -> str:
    indent = "\t"

    # handle empty structs
    if len(inst._cls.vars) == 0:
        return f"{inst._cls.key}()"

    lines = []
    lines.append(f"{inst._cls.key}(")

    for field_name, _ in inst._cls.ctype._fields_:
        field_value = getattr(inst, field_name, None)

        if isinstance(field_value, StructInstance):
            field_value = struct_instance_repr_recursive(field_value, depth + 1)

        lines.append(f"{indent * (depth + 1)}{field_name}={field_value},")

    lines.append(f"{indent * depth})")
    return "\n".join(lines)


class StructInstance:
    def __init__(self, cls: Struct, ctype):
        super().__setattr__("_cls", cls)

        # maintain a c-types object for the top-level instance the struct
        if not ctype:
            super().__setattr__("_ctype", cls.ctype())
        else:
            super().__setattr__("_ctype", ctype)

        # create Python attributes for each of the struct's variables
        for field, var in cls.vars.items():
            if isinstance(var.type, warp.codegen.Struct):
                self.__dict__[field] = StructInstance(var.type, getattr(self._ctype, field))
            elif isinstance(var.type, warp.types.array):
                self.__dict__[field] = None
            else:
                self.__dict__[field] = var.type()

    def __setattr__(self, name, value):
        if name not in self._cls.vars:
            raise RuntimeError(f"Trying to set Warp struct attribute that does not exist {name}")

        var = self._cls.vars[name]

        # update our ctype flat copy
        if isinstance(var.type, array):
            if value is None:
                # create array with null pointer
                setattr(self._ctype, name, array_t())
            else:
                # wp.array
                assert isinstance(value, array)
                assert types_equal(
                    value.dtype, var.type.dtype
                ), f"assign to struct member variable {name} failed, expected type {type_repr(var.type.dtype)}, got type {type_repr(value.dtype)}"
                setattr(self._ctype, name, value.__ctype__())

        elif isinstance(var.type, Struct):
            # assign structs by-value, otherwise we would have problematic cases transferring ownership
            # of the underlying ctypes data between shared Python struct instances

            if not isinstance(value, StructInstance):
                raise RuntimeError(
                    f"Trying to assign a non-structure value to a struct attribute with type: {self._cls.key}"
                )

            # destination attribution on self
            dest = getattr(self, name)

            if dest._cls.key is not value._cls.key:
                raise RuntimeError(
                    f"Trying to assign a structure of type {value._cls.key} to an attribute of {self._cls.key}"
                )

            # update all nested ctype vars by deep copy
            for n in dest._cls.vars:
                setattr(dest, n, getattr(value, n))

            # early return to avoid updating our Python StructInstance
            return

        elif issubclass(var.type, ctypes.Array):
            # vector/matrix type, e.g. vec3
            if value is None:
                setattr(self._ctype, name, var.type())
            elif types_equal(type(value), var.type):
                setattr(self._ctype, name, value)
            else:
                # conversion from list/tuple, ndarray, etc.
                setattr(self._ctype, name, var.type(value))

        else:
            # primitive type
            if value is None:
                # zero initialize
                setattr(self._ctype, name, var.type._type_())
            else:
                if hasattr(value, "_type_"):
                    # assigning warp type value (e.g.: wp.float32)
                    value = value.value
                # float16 needs conversion to uint16 bits
                if var.type == warp.float16:
                    setattr(self._ctype, name, float_to_half_bits(value))
                else:
                    setattr(self._ctype, name, value)

        # update Python instance
        super().__setattr__(name, value)

    def __ctype__(self):
        return self._ctype

    def __repr__(self):
        return struct_instance_repr_recursive(self, 0)

    # type description used in numpy structured arrays
    def numpy_dtype(self):
        return self._cls.numpy_dtype()

    # value usable in numpy structured arrays of .numpy_dtype(), e.g. (42, 13.37, [1.0, 2.0, 3.0])
    def numpy_value(self):
        npvalue = []
        for name, var in self._cls.vars.items():
            # get the attribute value
            value = getattr(self._ctype, name)

            if isinstance(var.type, array):
                # array_t
                npvalue.append(value.numpy_value())
            elif isinstance(var.type, Struct):
                # nested struct
                npvalue.append(value.numpy_value())
            elif issubclass(var.type, ctypes.Array):
                if len(var.type._shape_) == 1:
                    # vector
                    npvalue.append(list(value))
                else:
                    # matrix
                    npvalue.append([list(row) for row in value])
            else:
                # scalar
                if var.type == warp.float16:
                    npvalue.append(half_bits_to_float(value))
                else:
                    npvalue.append(value)

        return tuple(npvalue)


class Struct:
    def __init__(self, cls, key, module):
        self.cls = cls
        self.module = module
        self.key = key

        self.vars = {}
        annotations = get_annotations(self.cls)
        for label, type in annotations.items():
            self.vars[label] = Var(label, type)

        fields = []
        for label, var in self.vars.items():
            if isinstance(var.type, array):
                fields.append((label, array_t))
            elif isinstance(var.type, Struct):
                fields.append((label, var.type.ctype))
            elif issubclass(var.type, ctypes.Array):
                fields.append((label, var.type))
            else:
                fields.append((label, var.type._type_))

        class StructType(ctypes.Structure):
            # if struct is empty, add a dummy field to avoid launch errors on CPU device ("ffi_prep_cif failed")
            _fields_ = fields or [("_dummy_", ctypes.c_byte)]

        self.ctype = StructType

        # create default constructor (zero-initialize)
        self.default_constructor = warp.context.Function(
            func=None,
            key=self.key,
            namespace="",
            value_func=lambda *_: self,
            input_types={},
            initializer_list_func=lambda *_: False,
            native_func=make_full_qualified_name(self.cls),
        )

        # build a constructor that takes each param as a value
        input_types = {label: var.type for label, var in self.vars.items()}

        self.value_constructor = warp.context.Function(
            func=None,
            key=self.key,
            namespace="",
            value_func=lambda *_: self,
            input_types=input_types,
            initializer_list_func=lambda *_: False,
            native_func=make_full_qualified_name(self.cls),
        )

        self.default_constructor.add_overload(self.value_constructor)

        if module:
            module.register_struct(self)

    def __call__(self):
        """
        This function returns s = StructInstance(self)
        s uses self.cls as template.
        To enable autocomplete on s, we inherit from self.cls.
        For example,

        @wp.struct
        class A:
            # annotations
            ...

        The type annotations are inherited in A(), allowing autocomplete in kernels
        """
        # return StructInstance(self)

        class NewStructInstance(self.cls, StructInstance):
            def __init__(inst):
                StructInstance.__init__(inst, self, None)

        return NewStructInstance()

    def initializer(self):
        return self.default_constructor

    # return structured NumPy dtype, including field names, formats, and offsets
    def numpy_dtype(self):
        names = []
        formats = []
        offsets = []
        for name, var in self.vars.items():
            names.append(name)
            offsets.append(getattr(self.ctype, name).offset)
            if isinstance(var.type, array):
                # array_t
                formats.append(array_t.numpy_dtype())
            elif isinstance(var.type, Struct):
                # nested struct
                formats.append(var.type.numpy_dtype())
            elif issubclass(var.type, ctypes.Array):
                scalar_typestr = type_typestr(var.type._wp_scalar_type_)
                if len(var.type._shape_) == 1:
                    # vector
                    formats.append(f"{var.type._length_}{scalar_typestr}")
                else:
                    # matrix
                    formats.append(f"{var.type._shape_}{scalar_typestr}")
            else:
                # scalar
                formats.append(type_typestr(var.type))

        return {"names": names, "formats": formats, "offsets": offsets, "itemsize": ctypes.sizeof(self.ctype)}

    # constructs a Warp struct instance from a pointer to the ctype
    def from_ptr(self, ptr):
        if not ptr:
            raise RuntimeError("NULL pointer exception")

        # create a new struct instance
        instance = self()

        for name, var in self.vars.items():
            offset = getattr(self.ctype, name).offset
            if isinstance(var.type, array):
                # We could reconstruct wp.array from array_t, but it's problematic.
                # There's no guarantee that the original wp.array is still allocated and
                # no easy way to make a backref.
                # Instead, we just create a stub annotation, which is not a fully usable array object.
                setattr(instance, name, array(dtype=var.type.dtype, ndim=var.type.ndim))
            elif isinstance(var.type, Struct):
                # nested struct
                value = var.type.from_ptr(ptr + offset)
                setattr(instance, name, value)
            elif issubclass(var.type, ctypes.Array):
                # vector/matrix
                value = var.type.from_ptr(ptr + offset)
                setattr(instance, name, value)
            else:
                # scalar
                cvalue = ctypes.cast(ptr + offset, ctypes.POINTER(var.type._type_)).contents
                if var.type == warp.float16:
                    setattr(instance, name, half_bits_to_float(cvalue))
                else:
                    setattr(instance, name, cvalue.value)

        return instance


class Reference:
    def __init__(self, value_type):
        self.value_type = value_type


def is_reference(type):
    return isinstance(type, Reference)


def strip_reference(arg):
    if is_reference(arg):
        return arg.value_type
    else:
        return arg


def compute_type_str(base_name, template_params):
    if not template_params:
        return base_name

    def param2str(p):
        if isinstance(p, int):
            return str(p)
        elif hasattr(p, "_type_"):
            return f"wp::{p.__name__}"
        return p.__name__

    return f"{base_name}<{','.join(map(param2str, template_params))}>"


class Var:
    def __init__(self, label, type, requires_grad=False, constant=None, prefix=True):
        # convert built-in types to wp types
        if type == float:
            type = float32
        elif type == int:
            type = int32

        self.label = label
        self.type = type
        self.requires_grad = requires_grad
        self.constant = constant
        self.prefix = prefix

    def __str__(self):
        return self.label

    @staticmethod
    def type_to_ctype(t, value_type=False):
        if is_array(t):
            if hasattr(t.dtype, "_wp_generic_type_str_"):
                dtypestr = compute_type_str(f"wp::{t.dtype._wp_generic_type_str_}", t.dtype._wp_type_params_)
            elif isinstance(t.dtype, Struct):
                dtypestr = make_full_qualified_name(t.dtype.cls)
            elif t.dtype.__name__ in ("bool", "int", "float"):
                dtypestr = t.dtype.__name__
            else:
                dtypestr = f"wp::{t.dtype.__name__}"
            classstr = f"wp::{type(t).__name__}"
            return f"{classstr}_t<{dtypestr}>"
        elif isinstance(t, Struct):
            return make_full_qualified_name(t.cls)
        elif is_reference(t):
            if not value_type:
                return Var.type_to_ctype(t.value_type) + "*"
            else:
                return Var.type_to_ctype(t.value_type)
        elif hasattr(t, "_wp_generic_type_str_"):
            return compute_type_str(f"wp::{t._wp_generic_type_str_}", t._wp_type_params_)
        elif t.__name__ in ("bool", "int", "float"):
            return t.__name__
        else:
            return f"wp::{t.__name__}"

    def ctype(self, value_type=False):
        return Var.type_to_ctype(self.type, value_type)

    def emit(self, prefix: str = "var"):
        if self.prefix:
            return f"{prefix}_{self.label}"
        else:
            return self.label

    def emit_adj(self):
        return self.emit("adj")


class Block:
    # Represents a basic block of instructions, e.g.: list
    # of straight line instructions inside a for-loop or conditional

    def __init__(self):
        # list of statements inside this block
        self.body_forward = []
        self.body_replay = []
        self.body_reverse = []

        # list of vars declared in this block
        self.vars = []


def is_local_value(value) -> bool:
    """Check whether a variable is defined inside a kernel."""
    return isinstance(value, (warp.context.Function, Var))


class Adjoint:
    # Source code transformer, this class takes a Python function and
    # generates forward and backward SSA forms of the function instructions

    def __init__(
        adj,
        func,
        overload_annotations=None,
        is_user_function=False,
        skip_forward_codegen=False,
        skip_reverse_codegen=False,
        custom_reverse_mode=False,
        custom_reverse_num_input_args=-1,
        transformers: List[ast.NodeTransformer] = [],
    ):
        adj.func = func

        adj.is_user_function = is_user_function

        # whether the generation of the forward code is skipped for this function
        adj.skip_forward_codegen = skip_forward_codegen
        # whether the generation of the adjoint code is skipped for this function
        adj.skip_reverse_codegen = skip_reverse_codegen

        # extract name of source file
        adj.filename = inspect.getsourcefile(func) or "unknown source file"
        # get source file line number where function starts
        _, adj.fun_lineno = inspect.getsourcelines(func)

        # get function source code
        adj.source = inspect.getsource(func)
        # ensures that indented class methods can be parsed as kernels
        adj.source = textwrap.dedent(adj.source)

        adj.source_lines = adj.source.splitlines()

        # build AST and apply node transformers
        adj.tree = ast.parse(adj.source)
        adj.transformers = transformers
        for transformer in transformers:
            adj.tree = transformer.visit(adj.tree)

        adj.fun_name = adj.tree.body[0].name

        # for keeping track of line number in function code
        adj.lineno = None

        # whether the forward code shall be used for the reverse pass and a custom
        # function signature is applied to the reverse version of the function
        adj.custom_reverse_mode = custom_reverse_mode
        # the number of function arguments that pertain to the forward function
        # input arguments (i.e. the number of arguments that are not adjoint arguments)
        adj.custom_reverse_num_input_args = custom_reverse_num_input_args

        # parse argument types
        argspec = inspect.getfullargspec(func)

        # ensure all arguments are annotated
        if overload_annotations is None:
            # use source-level argument annotations
            if len(argspec.annotations) < len(argspec.args):
                raise WarpCodegenError(f"Incomplete argument annotations on function {adj.fun_name}")
            adj.arg_types = argspec.annotations
        else:
            # use overload argument annotations
            for arg_name in argspec.args:
                if arg_name not in overload_annotations:
                    raise WarpCodegenError(f"Incomplete overload annotations for function {adj.fun_name}")
            adj.arg_types = overload_annotations.copy()

        adj.args = []
        adj.symbols = {}

        for name, type in adj.arg_types.items():
            # skip return hint
            if name == "return":
                continue

            # add variable for argument
            arg = Var(name, type, False)
            adj.args.append(arg)

            # pre-populate symbol dictionary with function argument names
            # this is to avoid registering false references to overshadowed modules
            adj.symbols[name] = arg

        # There are cases where a same module might be rebuilt multiple times,
        # for example when kernels are nested inside of functions, or when
        # a kernel's launch raises an exception. Ideally we'd always want to
        # avoid rebuilding kernels but some corner cases seem to depend on it,
        # so we only avoid rebuilding kernels that errored out to give a chance
        # for unit testing errors being spit out from kernels.
        adj.skip_build = False

    # generate function ssa form and adjoint
    def build(adj, builder):
        if adj.skip_build:
            return

        adj.builder = builder

        adj.symbols = {}  # map from symbols to adjoint variables
        adj.variables = []  # list of local variables (in order)

        adj.return_var = None  # return type for function or kernel
        adj.loop_symbols = []  # symbols at the start of each loop

        # blocks
        adj.blocks = [Block()]
        adj.loop_blocks = []

        # holds current indent level
        adj.indentation = ""

        # used to generate new label indices
        adj.label_count = 0

        # update symbol map for each argument
        for a in adj.args:
            adj.symbols[a.label] = a

        # recursively evaluate function body
        try:
            adj.eval(adj.tree.body[0])
        except Exception as e:
            try:
                if isinstance(e, KeyError) and getattr(e.args[0], "__module__", None) == "ast":
                    msg = f'Syntax error: unsupported construct "ast.{e.args[0].__name__}"'
                else:
                    msg = "Error"
                lineno = adj.lineno + adj.fun_lineno
                line = adj.source_lines[adj.lineno]
                msg += f' while parsing function "{adj.fun_name}" at {adj.filename}:{lineno}:\n{line}\n'
                ex, data, traceback = sys.exc_info()
                e = ex(";".join([msg] + [str(a) for a in data.args])).with_traceback(traceback)
            finally:
                adj.skip_build = True
                raise e

        if builder is not None:
            for a in adj.args:
                if isinstance(a.type, Struct):
                    builder.build_struct_recursive(a.type)
                elif isinstance(a.type, warp.types.array) and isinstance(a.type.dtype, Struct):
                    builder.build_struct_recursive(a.type.dtype)

    # code generation methods
    def format_template(adj, template, input_vars, output_var):
        # output var is always the 0th index
        args = [output_var] + input_vars
        s = template.format(*args)

        return s

    # generates a list of formatted args
    def format_args(adj, prefix, args):
        arg_strs = []

        for a in args:
            if isinstance(a, warp.context.Function):
                # functions don't have a var_ prefix so strip it off here
                if prefix == "var":
                    arg_strs.append(a.key)
                else:
                    arg_strs.append(f"{prefix}_{a.key}")
            elif is_reference(a.type):
                arg_strs.append(f"{prefix}_{a}")
            elif isinstance(a, Var):
                arg_strs.append(a.emit(prefix))
            else:
                raise WarpCodegenTypeError(f"Arguments must be variables or functions, got {type(a)}")

        return arg_strs

    # generates argument string for a forward function call
    def format_forward_call_args(adj, args, use_initializer_list):
        arg_str = ", ".join(adj.format_args("var", args))
        if use_initializer_list:
            return f"{{{arg_str}}}"
        return arg_str

    # generates argument string for a reverse function call
    def format_reverse_call_args(
        adj,
        args_var,
        args,
        args_out,
        use_initializer_list,
        has_output_args=True,
        require_original_output_arg=False,
    ):
        formatted_var = adj.format_args("var", args_var)
        formatted_out = []
        if has_output_args and (require_original_output_arg or len(args_out) > 1):
            formatted_out = adj.format_args("var", args_out)
        formatted_var_adj = adj.format_args(
            "&adj" if use_initializer_list else "adj",
            args,
        )
        formatted_out_adj = adj.format_args("adj", args_out)

        if len(formatted_var_adj) == 0 and len(formatted_out_adj) == 0:
            # there are no adjoint arguments, so we don't need to call the reverse function
            return None

        if use_initializer_list:
            var_str = f"{{{', '.join(formatted_var)}}}"
            out_str = f"{{{', '.join(formatted_out)}}}"
            adj_str = f"{{{', '.join(formatted_var_adj)}}}"
            out_adj_str = ", ".join(formatted_out_adj)
            if len(args_out) > 1:
                arg_str = ", ".join([var_str, out_str, adj_str, out_adj_str])
            else:
                arg_str = ", ".join([var_str, adj_str, out_adj_str])
        else:
            arg_str = ", ".join(formatted_var + formatted_out + formatted_var_adj + formatted_out_adj)
        return arg_str

    def indent(adj):
        adj.indentation = adj.indentation + "    "

    def dedent(adj):
        adj.indentation = adj.indentation[:-4]

    def begin_block(adj):
        b = Block()

        # give block a unique id
        b.label = adj.label_count
        adj.label_count += 1

        adj.blocks.append(b)
        return b

    def end_block(adj):
        return adj.blocks.pop()

    def add_var(adj, type=None, constant=None):
        index = len(adj.variables)
        name = str(index)

        # allocate new variable
        v = Var(name, type=type, constant=constant)

        adj.variables.append(v)

        adj.blocks[-1].vars.append(v)

        return v

    # append a statement to the forward pass
    def add_forward(adj, statement, replay=None, skip_replay=False):
        adj.blocks[-1].body_forward.append(adj.indentation + statement)

        if not skip_replay:
            if replay:
                # if custom replay specified then output it
                adj.blocks[-1].body_replay.append(adj.indentation + replay)
            else:
                # by default just replay the original statement
                adj.blocks[-1].body_replay.append(adj.indentation + statement)

    # append a statement to the reverse pass
    def add_reverse(adj, statement):
        adj.blocks[-1].body_reverse.append(adj.indentation + statement)

    def add_constant(adj, n):
        output = adj.add_var(type=type(n), constant=n)
        return output

    def load(adj, var):
        if is_reference(var.type):
            var = adj.add_builtin_call("load", [var])
        return var

    def add_comp(adj, op_strings, left, comps):
        output = adj.add_var(builtins.bool)

        left = adj.load(left)
        s = output.emit() + " = " + ("(" * len(comps)) + left.emit() + " "

        prev_comp = None

        for op, comp in zip(op_strings, comps):
            comp_chainable = op_str_is_chainable(op)
            if comp_chainable and prev_comp:
                # We  restrict chaining to operands of the same type
                if prev_comp.type is comp.type:
                    prev_comp = adj.load(prev_comp)
                    comp = adj.load(comp)
                    s += "&& (" + prev_comp.emit() + " " + op + " " + comp.emit() + ")) "
                else:
                    raise WarpCodegenTypeError(
                        f"Cannot chain comparisons of unequal types: {prev_comp.type} {op} {comp.type}."
                    )
            else:
                comp = adj.load(comp)
                s += op + " " + comp.emit() + ") "

            prev_comp = comp

        s = s.rstrip() + ";"

        adj.add_forward(s)

        return output

    def add_bool_op(adj, op_string, exprs):
        exprs = [adj.load(expr) for expr in exprs]
        output = adj.add_var(builtins.bool)
        command = output.emit() + " = " + (" " + op_string + " ").join([expr.emit() for expr in exprs]) + ";"
        adj.add_forward(command)

        return output

    def resolve_func(adj, func, args, min_outputs, templates, kwds):
        arg_types = [strip_reference(a.type) for a in args if not isinstance(a, warp.context.Function)]

        if not func.is_builtin():
            # user-defined function
            overload = func.get_overload(arg_types)
            if overload is not None:
                return overload
        else:
            # if func is overloaded then perform overload resolution here
            # we validate argument types before they go to generated native code
            for f in func.overloads:
                # skip type checking for variadic functions
                if not f.variadic:
                    # check argument counts match are compatible (may be some default args)
                    if len(f.input_types) < len(args):
                        continue

                    def match_args(args, f):
                        # check argument types equal
                        for i, (arg_name, arg_type) in enumerate(f.input_types.items()):
                            # if arg type registered as Any, treat as
                            # template allowing any type to match
                            if arg_type == Any:
                                continue

                            # handle function refs as a special case
                            if arg_type == Callable and type(args[i]) is warp.context.Function:
                                continue

                            if arg_type == Reference and is_reference(args[i].type):
                                continue

                            # look for default values for missing args
                            if i >= len(args):
                                if arg_name not in f.defaults:
                                    return False
                            else:
                                # otherwise check arg type matches input variable type
                                if not types_equal(arg_type, strip_reference(args[i].type), match_generic=True):
                                    return False

                        return True

                    if not match_args(args, f):
                        continue

                # check output dimensions match expectations
                if min_outputs:
                    try:
                        value_type = f.value_func(args, kwds, templates)
                        if not hasattr(value_type, "__len__") or len(value_type) != min_outputs:
                            continue
                    except Exception:
                        # value func may fail if the user has given
                        # incorrect args, so we need to catch this
                        continue

                # found a match, use it
                return f

        # unresolved function, report error
        arg_types = []

        for x in args:
            if isinstance(x, Var):
                # shorten Warp primitive type names
                if isinstance(x.type, list):
                    if len(x.type) != 1:
                        raise WarpCodegenError("Argument must not be the result from a multi-valued function")
                    arg_type = x.type[0]
                else:
                    arg_type = x.type

                arg_types.append(type_repr(arg_type))

            if isinstance(x, warp.context.Function):
                arg_types.append("function")

        raise WarpCodegenError(
            f"Couldn't find function overload for '{func.key}' that matched inputs with types: [{', '.join(arg_types)}]"
        )

    def add_call(adj, func, args, min_outputs=None, templates=[], kwds=None):
        func = adj.resolve_func(func, args, min_outputs, templates, kwds)

        # push any default values onto args
        for i, (arg_name, arg_type) in enumerate(func.input_types.items()):
            if i >= len(args):
                if arg_name in func.defaults:
                    const = adj.add_constant(func.defaults[arg_name])
                    args.append(const)
                else:
                    break

        # if it is a user-function then build it recursively
        if not func.is_builtin():
            adj.builder.build_function(func)

        # evaluate the function type based on inputs
        arg_types = [strip_reference(a.type) for a in args if not isinstance(a, warp.context.Function)]
        return_type = func.value_func(arg_types, kwds, templates)

        func_name = compute_type_str(func.native_func, templates)
        param_types = list(func.input_types.values())

        use_initializer_list = func.initializer_list_func(args, templates)

        args_var = [
            adj.load(a)
            if not ((param_types[i] == Reference or param_types[i] == Callable) if i < len(param_types) else False)
            else a
            for i, a in enumerate(args)
        ]

        if return_type is None:
            # handles expression (zero output) functions, e.g.: void do_something();

            output = None
            output_list = []

            forward_call = (
                f"{func.namespace}{func_name}({adj.format_forward_call_args(args_var, use_initializer_list)});"
            )
            replay_call = forward_call
            if func.custom_replay_func is not None:
                replay_call = f"{func.namespace}replay_{func_name}({adj.format_forward_call_args(args_var, use_initializer_list)});"

        elif not isinstance(return_type, list) or len(return_type) == 1:
            # handle simple function (one output)

            if isinstance(return_type, list):
                return_type = return_type[0]
            output = adj.add_var(return_type)
            output_list = [output]

            forward_call = f"var_{output} = {func.namespace}{func_name}({adj.format_forward_call_args(args_var, use_initializer_list)});"
            replay_call = forward_call
            if func.custom_replay_func is not None:
                replay_call = f"var_{output} = {func.namespace}replay_{func_name}({adj.format_forward_call_args(args_var, use_initializer_list)});"

        else:
            # handle multiple value functions

            output = [adj.add_var(v) for v in return_type]
            output_list = output

            forward_call = (
                f"{func.namespace}{func_name}({adj.format_forward_call_args(args_var + output, use_initializer_list)});"
            )
            replay_call = forward_call

        if func.skip_replay:
            adj.add_forward(forward_call, replay="// " + replay_call)
        else:
            adj.add_forward(forward_call, replay=replay_call)

        if not func.missing_grad and len(args):
            reverse_has_output_args = (
                func.require_original_output_arg or len(output_list) > 1
            ) and func.custom_grad_func is None
            arg_str = adj.format_reverse_call_args(
                args_var,
                args,
                output_list,
                use_initializer_list,
                has_output_args=reverse_has_output_args,
                require_original_output_arg=func.require_original_output_arg,
            )
            if arg_str is not None:
                reverse_call = f"{func.namespace}adj_{func.native_func}({arg_str});"
                adj.add_reverse(reverse_call)

        return output

    def add_builtin_call(adj, func_name, args, min_outputs=None, templates=[], kwds=None):
        func = warp.context.builtin_functions[func_name]
        return adj.add_call(func, args, min_outputs, templates, kwds)

    def add_return(adj, var):
        if var is None or len(var) == 0:
            adj.add_forward("return;", f"goto label{adj.label_count};")
        elif len(var) == 1:
            adj.add_forward(f"return {var[0].emit()};", f"goto label{adj.label_count};")
            adj.add_reverse("adj_" + str(var[0]) + " += adj_ret;")
        else:
            for i, v in enumerate(var):
                adj.add_forward(f"ret_{i} = {v.emit()};")
                adj.add_reverse(f"adj_{v} += adj_ret_{i};")
            adj.add_forward("return;", f"goto label{adj.label_count};")

        adj.add_reverse(f"label{adj.label_count}:;")

        adj.label_count += 1

    # define an if statement
    def begin_if(adj, cond):
        cond = adj.load(cond)
        adj.add_forward(f"if ({cond.emit()}) {{")
        adj.add_reverse("}")

        adj.indent()

    def end_if(adj, cond):
        adj.dedent()

        adj.add_forward("}")
        cond = adj.load(cond)
        adj.add_reverse(f"if ({cond.emit()}) {{")

    def begin_else(adj, cond):
        cond = adj.load(cond)
        adj.add_forward(f"if (!{cond.emit()}) {{")
        adj.add_reverse("}")

        adj.indent()

    def end_else(adj, cond):
        adj.dedent()

        adj.add_forward("}")
        cond = adj.load(cond)
        adj.add_reverse(f"if (!{cond.emit()}) {{")

    # define a for-loop
    def begin_for(adj, iter):
        cond_block = adj.begin_block()
        adj.loop_blocks.append(cond_block)
        adj.add_forward(f"for_start_{cond_block.label}:;")
        adj.indent()

        # evaluate cond
        adj.add_forward(f"if (iter_cmp({iter.emit()}) == 0) goto for_end_{cond_block.label};")

        # evaluate iter
        val = adj.add_builtin_call("iter_next", [iter])

        adj.begin_block()

        return val

    def end_for(adj, iter):
        body_block = adj.end_block()
        cond_block = adj.end_block()
        adj.loop_blocks.pop()

        ####################
        # forward pass

        for i in cond_block.body_forward:
            adj.blocks[-1].body_forward.append(i)

        for i in body_block.body_forward:
            adj.blocks[-1].body_forward.append(i)

        adj.add_forward(f"goto for_start_{cond_block.label};", skip_replay=True)

        adj.dedent()
        adj.add_forward(f"for_end_{cond_block.label}:;", skip_replay=True)

        ####################
        # reverse pass

        reverse = []

        # reverse iterator
        reverse.append(adj.indentation + f"{iter.emit()} = wp::iter_reverse({iter.emit()});")

        for i in cond_block.body_forward:
            reverse.append(i)

        # zero adjoints
        for i in body_block.vars:
            reverse.append(adj.indentation + f"\t{i.emit_adj()} = {{}};")

        # replay
        for i in body_block.body_replay:
            reverse.append(i)

        # reverse
        for i in reversed(body_block.body_reverse):
            reverse.append(i)

        reverse.append(adj.indentation + f"\tgoto for_start_{cond_block.label};")
        reverse.append(adj.indentation + f"for_end_{cond_block.label}:;")

        adj.blocks[-1].body_reverse.extend(reversed(reverse))

    # define a while loop
    def begin_while(adj, cond):
        # evaluate condition in its own block
        # so we can control replay
        cond_block = adj.begin_block()
        adj.loop_blocks.append(cond_block)
        cond_block.body_forward.append(f"while_start_{cond_block.label}:;")

        c = adj.eval(cond)

        cond_block.body_forward.append(f"if (({c.emit()}) == false) goto while_end_{cond_block.label};")

        # being block around loop
        adj.begin_block()
        adj.indent()

    def end_while(adj):
        adj.dedent()
        body_block = adj.end_block()
        cond_block = adj.end_block()
        adj.loop_blocks.pop()

        ####################
        # forward pass

        for i in cond_block.body_forward:
            adj.blocks[-1].body_forward.append(i)

        for i in body_block.body_forward:
            adj.blocks[-1].body_forward.append(i)

        adj.blocks[-1].body_forward.append(f"goto while_start_{cond_block.label};")
        adj.blocks[-1].body_forward.append(f"while_end_{cond_block.label}:;")

        ####################
        # reverse pass
        reverse = []

        # cond
        for i in cond_block.body_forward:
            reverse.append(i)

        # zero adjoints of local vars
        for i in body_block.vars:
            reverse.append(f"{i.emit_adj()} = {{}};")

        # replay
        for i in body_block.body_replay:
            reverse.append(i)

        # reverse
        for i in reversed(body_block.body_reverse):
            reverse.append(i)

        reverse.append(f"goto while_start_{cond_block.label};")
        reverse.append(f"while_end_{cond_block.label}:;")

        # output
        adj.blocks[-1].body_reverse.extend(reversed(reverse))

    def emit_FunctionDef(adj, node):
        for f in node.body:
            adj.eval(f)

        if adj.return_var is not None and len(adj.return_var) == 1:
            if not isinstance(node.body[-1], ast.Return):
                adj.add_forward("return {};", skip_replay=True)

    def emit_If(adj, node):
        if len(node.body) == 0:
            return None

        # eval condition
        cond = adj.eval(node.test)

        # save symbol map
        symbols_prev = adj.symbols.copy()

        # eval body
        adj.begin_if(cond)

        for stmt in node.body:
            adj.eval(stmt)

        adj.end_if(cond)

        # detect existing symbols with conflicting definitions (variables assigned inside the branch)
        # and resolve with a phi (select) function
        for items in symbols_prev.items():
            sym = items[0]
            var1 = items[1]
            var2 = adj.symbols[sym]

            if var1 != var2:
                # insert a phi function that selects var1, var2 based on cond
                out = adj.add_builtin_call("select", [cond, var1, var2])
                adj.symbols[sym] = out

        symbols_prev = adj.symbols.copy()

        # evaluate 'else' statement as if (!cond)
        if len(node.orelse) > 0:
            adj.begin_else(cond)

            for stmt in node.orelse:
                adj.eval(stmt)

            adj.end_else(cond)

        # detect existing symbols with conflicting definitions (variables assigned inside the else)
        # and resolve with a phi (select) function
        for items in symbols_prev.items():
            sym = items[0]
            var1 = items[1]
            var2 = adj.symbols[sym]

            if var1 != var2:
                # insert a phi function that selects var1, var2 based on cond
                # note the reversed order of vars since we want to use !cond as our select
                out = adj.add_builtin_call("select", [cond, var2, var1])
                adj.symbols[sym] = out

    def emit_Compare(adj, node):
        # node.left, node.ops (list of ops), node.comparators (things to compare to)
        # e.g. (left ops[0] node.comparators[0]) ops[1] node.comparators[1]

        left = adj.eval(node.left)
        comps = [adj.eval(comp) for comp in node.comparators]
        op_strings = [builtin_operators[type(op)] for op in node.ops]

        return adj.add_comp(op_strings, left, comps)

    def emit_BoolOp(adj, node):
        # op, expr list values

        op = node.op
        if isinstance(op, ast.And):
            func = "&&"
        elif isinstance(op, ast.Or):
            func = "||"
        else:
            raise WarpCodegenKeyError(f"Op {op} is not supported")

        return adj.add_bool_op(func, [adj.eval(expr) for expr in node.values])

    def emit_Name(adj, node):
        # lookup symbol, if it has already been assigned to a variable then return the existing mapping
        if node.id in adj.symbols:
            return adj.symbols[node.id]

        # try and resolve the name using the function's globals context (used to lookup constants + functions)
        obj = adj.func.__globals__.get(node.id)

        if obj is None:
            # Lookup constant in captured contents
            capturedvars = dict(
                zip(adj.func.__code__.co_freevars, [c.cell_contents for c in (adj.func.__closure__ or [])])
            )
            obj = capturedvars.get(str(node.id), None)

        if obj is None:
            raise WarpCodegenKeyError("Referencing undefined symbol: " + str(node.id))

        if warp.types.is_value(obj):
            # evaluate constant
            out = adj.add_constant(obj)
            adj.symbols[node.id] = out
            return out

        # the named object is either a function, class name, or module
        # pass it back to the caller for processing
        return obj

    @staticmethod
    def resolve_type_attribute(var_type: type, attr: str):
        if isinstance(var_type, type) and type_is_value(var_type):
            if attr == "dtype":
                return type_scalar_type(var_type)
            elif attr == "length":
                return type_length(var_type)

        return getattr(var_type, attr, None)

    def vector_component_index(adj, component, vector_type):
        if len(component) != 1:
            raise WarpCodegenAttributeError(f"Vector swizzle must be single character, got .{component}")

        dim = vector_type._shape_[0]
        swizzles = "xyzw"[0:dim]
        if component not in swizzles:
            raise WarpCodegenAttributeError(
                f"Vector swizzle for {vector_type} must be one of {swizzles}, got {component}"
            )

        index = swizzles.index(component)
        index = adj.add_constant(index)
        return index

    @staticmethod
    def is_differentiable_value_type(var_type):
        # checks that the argument type is a value type (i.e, not an array)
        # possibly holding differentiable values (for which gradients must be accumulated)
        return type_scalar_type(var_type) in float_types or isinstance(var_type, Struct)

    def emit_Attribute(adj, node):
        if hasattr(node, "is_adjoint"):
            node.value.is_adjoint = True

        aggregate = adj.eval(node.value)

        try:
            if isinstance(aggregate, types.ModuleType) or isinstance(aggregate, type):
                out = getattr(aggregate, node.attr)

                if warp.types.is_value(out):
                    return adj.add_constant(out)

                return out

            if hasattr(node, "is_adjoint"):
                # create a Var that points to the struct attribute, i.e.: directly generates `struct.attr` when used
                attr_name = aggregate.label + "." + node.attr
                attr_type = aggregate.type.vars[node.attr].type

                return Var(attr_name, attr_type)

            aggregate_type = strip_reference(aggregate.type)

            # reading a vector component
            if type_is_vector(aggregate_type):
                index = adj.vector_component_index(node.attr, aggregate_type)

                return adj.add_builtin_call("extract", [aggregate, index])

            else:
                attr_type = Reference(aggregate_type.vars[node.attr].type)
                attr = adj.add_var(attr_type)

                if is_reference(aggregate.type):
                    adj.add_forward(f"{attr.emit()} = &({aggregate.emit()}->{node.attr});")
                else:
                    adj.add_forward(f"{attr.emit()} = &({aggregate.emit()}.{node.attr});")

                if adj.is_differentiable_value_type(strip_reference(attr_type)):
                    adj.add_reverse(f"{aggregate.emit_adj()}.{node.attr} += {attr.emit_adj()};")
                else:
                    adj.add_reverse(f"{aggregate.emit_adj()}.{node.attr} = {attr.emit_adj()};")

                return attr

        except (KeyError, AttributeError):
            # Try resolving as type attribute
            aggregate_type = strip_reference(aggregate.type) if isinstance(aggregate, Var) else aggregate

            type_attribute = adj.resolve_type_attribute(aggregate_type, node.attr)
            if type_attribute is not None:
                return type_attribute

            if isinstance(aggregate, Var):
                raise WarpCodegenAttributeError(
                    f"Error, `{node.attr}` is not an attribute of '{node.value.id}' ({type_repr(aggregate.type)})"
                )
            raise WarpCodegenAttributeError(f"Error, `{node.attr}` is not an attribute of '{aggregate}'")

    def emit_String(adj, node):
        # string constant
        return adj.add_constant(node.s)

    def emit_Num(adj, node):
        # lookup constant, if it has already been assigned then return existing var
        key = (node.n, type(node.n))

        if key in adj.symbols:
            return adj.symbols[key]
        else:
            out = adj.add_constant(node.n)
            adj.symbols[key] = out
            return out

    def emit_Ellipsis(adj, node):
        # stubbed @wp.native_func
        return

    def emit_NameConstant(adj, node):
        if node.value:
            return adj.add_constant(True)
        elif node.value is None:
            raise WarpCodegenTypeError("None type unsupported")
        else:
            return adj.add_constant(False)

    def emit_Constant(adj, node):
        if isinstance(node, ast.Str):
            return adj.emit_String(node)
        elif isinstance(node, ast.Num):
            return adj.emit_Num(node)
        elif isinstance(node, ast.Ellipsis):
            return adj.emit_Ellipsis(node)
        else:
            assert isinstance(node, ast.NameConstant)
            return adj.emit_NameConstant(node)

    def emit_BinOp(adj, node):
        # evaluate binary operator arguments
        left = adj.eval(node.left)
        right = adj.eval(node.right)

        name = builtin_operators[type(node.op)]

        return adj.add_builtin_call(name, [left, right])

    def emit_UnaryOp(adj, node):
        # evaluate unary op arguments
        arg = adj.eval(node.operand)

        name = builtin_operators[type(node.op)]

        return adj.add_builtin_call(name, [arg])

    def materialize_redefinitions(adj, symbols):
        # detect symbols with conflicting definitions (assigned inside the for loop)
        for items in symbols.items():
            sym = items[0]
            var1 = items[1]
            var2 = adj.symbols[sym]

            if var1 != var2:
                if warp.config.verbose and not adj.custom_reverse_mode:
                    lineno = adj.lineno + adj.fun_lineno
                    line = adj.source_lines[adj.lineno]
                    msg = f'Warning: detected mutated variable {sym} during a dynamic for-loop in function "{adj.fun_name}" at {adj.filename}:{lineno}: this may not be a differentiable operation.\n{line}\n'
                    print(msg)

                if var1.constant is not None:
                    raise WarpCodegenError(
                        f"Error mutating a constant {sym} inside a dynamic loop, use the following syntax: pi = float(3.141) to declare a dynamic variable"
                    )

                # overwrite the old variable value (violates SSA)
                adj.add_builtin_call("assign", [var1, var2])

                # reset the symbol to point to the original variable
                adj.symbols[sym] = var1

    def emit_While(adj, node):
        adj.begin_while(node.test)

        adj.loop_symbols.append(adj.symbols.copy())

        # eval body
        for s in node.body:
            adj.eval(s)

        adj.materialize_redefinitions(adj.loop_symbols[-1])
        adj.loop_symbols.pop()

        adj.end_while()

    def eval_num(adj, a):
        if isinstance(a, ast.Num):
            return True, a.n
        if isinstance(a, ast.UnaryOp) and isinstance(a.op, ast.USub) and isinstance(a.operand, ast.Num):
            return True, -a.operand.n

        # try and resolve the expression to an object
        # e.g.: wp.constant in the globals scope
        obj, _ = adj.resolve_static_expression(a)

        if isinstance(obj, Var) and obj.constant is not None:
            obj = obj.constant

        return warp.types.is_int(obj), obj

    # detects whether a loop contains a break (or continue) statement
    def contains_break(adj, body):
        for s in body:
            if isinstance(s, ast.Break):
                return True
            elif isinstance(s, ast.Continue):
                return True
            elif isinstance(s, ast.If):
                if adj.contains_break(s.body):
                    return True
                if adj.contains_break(s.orelse):
                    return True
            else:
                # note that nested for or while loops containing a break statement
                # do not affect the current loop
                pass

        return False

    # returns a constant range() if unrollable, otherwise None
    def get_unroll_range(adj, loop):
        if (
            not isinstance(loop.iter, ast.Call)
            or not isinstance(loop.iter.func, ast.Name)
            or loop.iter.func.id != "range"
            or len(loop.iter.args) == 0
            or len(loop.iter.args) > 3
        ):
            return None

        # if all range() arguments are numeric constants we will unroll
        # note that this only handles trivial constants, it will not unroll
        # constant compile-time expressions e.g.: range(0, 3*2)

        # Evaluate the arguments and check that they are numeric constants
        # It is important to do that in one pass, so that if evaluating these arguments have side effects
        # the code does not get generated more than once
        range_args = [adj.eval_num(arg) for arg in loop.iter.args]
        arg_is_numeric, arg_values = zip(*range_args)

        if all(arg_is_numeric):
            # All argument are numeric constants

            # range(end)
            if len(loop.iter.args) == 1:
                start = 0
                end = arg_values[0]
                step = 1

            # range(start, end)
            elif len(loop.iter.args) == 2:
                start = arg_values[0]
                end = arg_values[1]
                step = 1

            # range(start, end, step)
            elif len(loop.iter.args) == 3:
                start = arg_values[0]
                end = arg_values[1]
                step = arg_values[2]

            # test if we're above max unroll count
            max_iters = abs(end - start) // abs(step)
            max_unroll = adj.builder.options["max_unroll"]

            ok_to_unroll = True

            if max_iters > max_unroll:
                if warp.config.verbose:
                    print(
                        f"Warning: fixed-size loop count of {max_iters} is larger than the module 'max_unroll' limit of {max_unroll}, will generate dynamic loop."
                    )
                ok_to_unroll = False

            elif adj.contains_break(loop.body):
                if warp.config.verbose:
                    print("Warning: 'break' or 'continue' found in loop body, will generate dynamic loop.")
                ok_to_unroll = False

            if ok_to_unroll:
                return range(start, end, step)

        # Unroll is not possible, range needs to be valuated dynamically
        range_call = adj.add_builtin_call(
            "range",
            [adj.add_constant(val) if is_numeric else val for is_numeric, val in range_args],
        )
        return range_call

    def emit_For(adj, node):
        # try and unroll simple range() statements that use constant args
        unroll_range = adj.get_unroll_range(node)

        if isinstance(unroll_range, range):
            for i in unroll_range:
                const_iter = adj.add_constant(i)
                var_iter = adj.add_builtin_call("int", [const_iter])
                adj.symbols[node.target.id] = var_iter

                # eval body
                for s in node.body:
                    adj.eval(s)

        # otherwise generate a dynamic loop
        else:
            # evaluate the Iterable -- only if not previously evaluated when trying to unroll
            if unroll_range is not None:
                # Range has already been evaluated when trying to unroll, do not re-evaluate
                iter = unroll_range
            else:
                iter = adj.eval(node.iter)

            adj.symbols[node.target.id] = adj.begin_for(iter)

            # for loops should be side-effect free, here we store a copy
            adj.loop_symbols.append(adj.symbols.copy())

            # eval body
            for s in node.body:
                adj.eval(s)

            adj.materialize_redefinitions(adj.loop_symbols[-1])
            adj.loop_symbols.pop()

            adj.end_for(iter)

    def emit_Break(adj, node):
        adj.materialize_redefinitions(adj.loop_symbols[-1])

        adj.add_forward(f"goto for_end_{adj.loop_blocks[-1].label};")

    def emit_Continue(adj, node):
        adj.materialize_redefinitions(adj.loop_symbols[-1])

        adj.add_forward(f"goto for_start_{adj.loop_blocks[-1].label};")

    def emit_Expr(adj, node):
        return adj.eval(node.value)

    def check_tid_in_func_error(adj, node):
        if adj.is_user_function:
            if hasattr(node.func, "attr") and node.func.attr == "tid":
                lineno = adj.lineno + adj.fun_lineno
                line = adj.source_lines[adj.lineno]
                raise WarpCodegenError(
                    "tid() may only be called from a Warp kernel, not a Warp function. "
                    "Instead, obtain the indices from a @wp.kernel and pass them as "
                    f"arguments to the function {adj.fun_name}, {adj.filename}:{lineno}:\n{line}\n"
                )

    def emit_Call(adj, node):
        adj.check_tid_in_func_error(node)

        # try and lookup function in globals by
        # resolving path (e.g.: module.submodule.attr)
        func, path = adj.resolve_static_expression(node.func)
        templates = []

        if not isinstance(func, warp.context.Function):
            if len(path) == 0:
                raise WarpCodegenError(f"Unknown function or operator: '{node.func.func.id}'")

            attr = path[-1]
            caller = func
            func = None

            # try and lookup function name in builtins (e.g.: using `dot` directly without wp prefix)
            if attr in warp.context.builtin_functions:
                func = warp.context.builtin_functions[attr]

            # vector class type e.g.: wp.vec3f constructor
            if func is None and hasattr(caller, "_wp_generic_type_str_"):
                templates = caller._wp_type_params_
                func = warp.context.builtin_functions.get(caller._wp_constructor_)

            # scalar class type e.g.: wp.int8 constructor
            if func is None and hasattr(caller, "__name__") and caller.__name__ in warp.context.builtin_functions:
                func = warp.context.builtin_functions.get(caller.__name__)

            # struct constructor
            if func is None and isinstance(caller, Struct):
                adj.builder.build_struct_recursive(caller)
                func = caller.initializer()

            if func is None:
                raise WarpCodegenError(
                    f"Could not find function {'.'.join(path)} as a built-in or user-defined function. Note that user functions must be annotated with a @wp.func decorator to be called from a kernel."
                )

        args = []

        # eval all arguments
        for arg in node.args:
            var = adj.eval(arg)
            if not is_local_value(var):
                raise RuntimeError(
                    "Cannot reference a global variable from a kernel unless `wp.constant()` is being used"
                )
            args.append(var)

        # eval all keyword args
        def kwval(kw):
            if isinstance(kw.value, ast.Num):
                return kw.value.n
            elif isinstance(kw.value, ast.Tuple):
                arg_is_numeric, arg_values = zip(*(adj.eval_num(e) for e in kw.value.elts))
                if not all(arg_is_numeric):
                    raise WarpCodegenError(
                        f"All elements of the tuple keyword argument '{kw.name}' must be numeric constants, got '{arg_values}'"
                    )
                return arg_values
            else:
                return adj.resolve_static_expression(kw.value)[0]

        kwds = {kw.arg: kwval(kw) for kw in node.keywords}

        # get expected return count, e.g.: for multi-assignment
        min_outputs = None
        if hasattr(node, "expects"):
            min_outputs = node.expects

        # add var with value type from the function
        out = adj.add_call(func=func, args=args, kwds=kwds, templates=templates, min_outputs=min_outputs)
        return out

    def emit_Index(adj, node):
        # the ast.Index node appears in 3.7 versions
        # when performing array slices, e.g.: x = arr[i]
        # but in version 3.8 and higher it does not appear

        if hasattr(node, "is_adjoint"):
            node.value.is_adjoint = True

        return adj.eval(node.value)

    def emit_Subscript(adj, node):
        if hasattr(node.value, "attr") and node.value.attr == "adjoint":
            # handle adjoint of a variable, i.e. wp.adjoint[var]
            node.slice.is_adjoint = True
            var = adj.eval(node.slice)
            var_name = var.label
            var = Var(f"adj_{var_name}", type=var.type, constant=None, prefix=False)
            return var

        target = adj.eval(node.value)
        if not is_local_value(target):
            raise RuntimeError(
                "Cannot reference a global variable from a kernel unless `wp.constant()` is being used"
            )

        indices = []

        if isinstance(node.slice, ast.Tuple):
            # handles the x[i,j] case (Python 3.8.x upward)
            for arg in node.slice.elts:
                var = adj.eval(arg)
                indices.append(var)

        elif isinstance(node.slice, ast.Index) and isinstance(node.slice.value, ast.Tuple):
            # handles the x[i,j] case (Python 3.7.x)
            for arg in node.slice.value.elts:
                var = adj.eval(arg)
                indices.append(var)
        else:
            # simple expression, e.g.: x[i]
            var = adj.eval(node.slice)
            indices.append(var)

        target_type = strip_reference(target.type)
        if is_array(target_type):
            if len(indices) == target_type.ndim:
                # handles array loads (where each dimension has an index specified)
                out = adj.add_builtin_call("address", [target, *indices])
            else:
                # handles array views (fewer indices than dimensions)
                out = adj.add_builtin_call("view", [target, *indices])

        else:
            # handles non-array type indexing, e.g: vec3, mat33, etc
            out = adj.add_builtin_call("extract", [target, *indices])

        return out

    def emit_Assign(adj, node):
        if len(node.targets) != 1:
            raise WarpCodegenError("Assigning the same value to multiple variables is not supported")

        lhs = node.targets[0]

        # handle the case where we are assigning multiple output variables
        if isinstance(lhs, ast.Tuple):
            # record the expected number of outputs on the node
            # we do this so we can decide which function to
            # call based on the number of expected outputs
            if isinstance(node.value, ast.Call):
                node.value.expects = len(lhs.elts)

            # evaluate values
            if isinstance(node.value, ast.Tuple):
                out = [adj.eval(v) for v in node.value.elts]
            else:
                out = adj.eval(node.value)

            names = []
            for v in lhs.elts:
                if isinstance(v, ast.Name):
                    names.append(v.id)
                else:
                    raise WarpCodegenError(
                        "Multiple return functions can only assign to simple variables, e.g.: x, y = func()"
                    )

            if len(names) != len(out):
                raise WarpCodegenError(
                    f"Multiple return functions need to receive all their output values, incorrect number of values to unpack (expected {len(out)}, got {len(names)})"
                )

            for name, rhs in zip(names, out):
                if name in adj.symbols:
                    if not types_equal(rhs.type, adj.symbols[name].type):
                        raise WarpCodegenTypeError(
                            f"Error, assigning to existing symbol {name} ({adj.symbols[name].type}) with different type ({rhs.type})"
                        )

                adj.symbols[name] = rhs

        # handles the case where we are assigning to an array index (e.g.: arr[i] = 2.0)
        elif isinstance(lhs, ast.Subscript):
            if hasattr(lhs.value, "attr") and lhs.value.attr == "adjoint":
                # handle adjoint of a variable, i.e. wp.adjoint[var]
                lhs.slice.is_adjoint = True
                src_var = adj.eval(lhs.slice)
                var = Var(f"adj_{src_var.label}", type=src_var.type, constant=None, prefix=False)
                value = adj.eval(node.value)
                adj.add_forward(f"{var.emit()} = {value.emit()};")
                return

            target = adj.eval(lhs.value)
            value = adj.eval(node.value)
            if not is_local_value(value):
                raise RuntimeError(
                    "Cannot reference a global variable from a kernel unless `wp.constant()` is being used"
                )

            slice = lhs.slice
            indices = []

            if isinstance(slice, ast.Tuple):
                # handles the x[i, j] case (Python 3.8.x upward)
                for arg in slice.elts:
                    var = adj.eval(arg)
                    indices.append(var)
            elif isinstance(slice, ast.Index) and isinstance(slice.value, ast.Tuple):
                # handles the x[i, j] case (Python 3.7.x)
                for arg in slice.value.elts:
                    var = adj.eval(arg)
                    indices.append(var)
            else:
                # simple expression, e.g.: x[i]
                var = adj.eval(slice)
                indices.append(var)

            target_type = strip_reference(target.type)

            if is_array(target_type):
                adj.add_builtin_call("array_store", [target, *indices, value])

            elif type_is_vector(target_type) or type_is_matrix(target_type):
                if is_reference(target.type):
                    attr = adj.add_builtin_call("indexref", [target, *indices])
                else:
                    attr = adj.add_builtin_call("index", [target, *indices])

                adj.add_builtin_call("store", [attr, value])

                if warp.config.verbose and not adj.custom_reverse_mode:
                    lineno = adj.lineno + adj.fun_lineno
                    line = adj.source_lines[adj.lineno]
                    node_source = adj.get_node_source(lhs.value)
                    print(
                        f"Warning: mutating {node_source} in function {adj.fun_name} at {adj.filename}:{lineno}: this is a non-differentiable operation.\n{line}\n"
                    )

            else:
                raise WarpCodegenError("Can only subscript assign array, vector, and matrix types")

        elif isinstance(lhs, ast.Name):
            # symbol name
            name = lhs.id

            # evaluate rhs
            rhs = adj.eval(node.value)

            # check type matches if symbol already defined
            if name in adj.symbols:
                if not types_equal(strip_reference(rhs.type), adj.symbols[name].type):
                    raise WarpCodegenTypeError(
                        f"Error, assigning to existing symbol {name} ({adj.symbols[name].type}) with different type ({rhs.type})"
                    )

            # handle simple assignment case (a = b), where we generate a value copy rather than reference
            if isinstance(node.value, ast.Name) or is_reference(rhs.type):
                out = adj.add_builtin_call("copy", [rhs])
            else:
                out = rhs

            # update symbol map (assumes lhs is a Name node)
            adj.symbols[name] = out

        elif isinstance(lhs, ast.Attribute):
            rhs = adj.eval(node.value)
            aggregate = adj.eval(lhs.value)
            aggregate_type = strip_reference(aggregate.type)

            # assigning to a vector component
            if type_is_vector(aggregate_type):
                index = adj.vector_component_index(lhs.attr, aggregate_type)

                if is_reference(aggregate.type):
                    attr = adj.add_builtin_call("indexref", [aggregate, index])
                else:
                    attr = adj.add_builtin_call("index", [aggregate, index])

                adj.add_builtin_call("store", [attr, rhs])

            else:
                attr = adj.emit_Attribute(lhs)
                if is_reference(attr.type):
                    adj.add_builtin_call("store", [attr, rhs])
                else:
                    adj.add_builtin_call("assign", [attr, rhs])

                if warp.config.verbose and not adj.custom_reverse_mode:
                    lineno = adj.lineno + adj.fun_lineno
                    line = adj.source_lines[adj.lineno]
                    msg = f'Warning: detected mutated struct {attr.label} during function "{adj.fun_name}" at {adj.filename}:{lineno}: this is a non-differentiable operation.\n{line}\n'
                    print(msg)

        else:
            raise WarpCodegenError("Error, unsupported assignment statement.")

    def emit_Return(adj, node):
        if node.value is None:
            var = None
        elif isinstance(node.value, ast.Tuple):
            var = tuple(adj.eval(arg) for arg in node.value.elts)
        else:
            var = (adj.eval(node.value),)

        if adj.return_var is not None:
            old_ctypes = tuple(v.ctype(value_type=True) for v in adj.return_var)
            new_ctypes = tuple(v.ctype(value_type=True) for v in var)
            if old_ctypes != new_ctypes:
                raise WarpCodegenTypeError(
                    f"Error, function returned different types, previous: [{', '.join(old_ctypes)}], new [{', '.join(new_ctypes)}]"
                )

        if var is not None:
            adj.return_var = tuple()
            for ret in var:
                if is_reference(ret.type):
                    ret = adj.add_builtin_call("copy", [ret])
                adj.return_var += (ret,)

        adj.add_return(adj.return_var)

    def emit_AugAssign(adj, node):
        # replace augmented assignment with assignment statement + binary op
        new_node = ast.Assign(targets=[node.target], value=ast.BinOp(node.target, node.op, node.value))
        adj.eval(new_node)

    def emit_Tuple(adj, node):
        # LHS for expressions, such as i, j, k = 1, 2, 3
        for elem in node.elts:
            adj.eval(elem)

    def emit_Pass(adj, node):
        pass

    node_visitors = {
        ast.FunctionDef: emit_FunctionDef,
        ast.If: emit_If,
        ast.Compare: emit_Compare,
        ast.BoolOp: emit_BoolOp,
        ast.Name: emit_Name,
        ast.Attribute: emit_Attribute,
        ast.Str: emit_String,  # Deprecated in 3.8; use Constant
        ast.Num: emit_Num,  # Deprecated in 3.8; use Constant
        ast.NameConstant: emit_NameConstant,  # Deprecated in 3.8; use Constant
        ast.Constant: emit_Constant,
        ast.BinOp: emit_BinOp,
        ast.UnaryOp: emit_UnaryOp,
        ast.While: emit_While,
        ast.For: emit_For,
        ast.Break: emit_Break,
        ast.Continue: emit_Continue,
        ast.Expr: emit_Expr,
        ast.Call: emit_Call,
        ast.Index: emit_Index,  # Deprecated in 3.8; Use the index value directly instead.
        ast.Subscript: emit_Subscript,
        ast.Assign: emit_Assign,
        ast.Return: emit_Return,
        ast.AugAssign: emit_AugAssign,
        ast.Tuple: emit_Tuple,
        ast.Pass: emit_Pass,
        ast.Ellipsis: emit_Ellipsis,
    }

    def eval(adj, node):
        if hasattr(node, "lineno"):
            adj.set_lineno(node.lineno - 1)

        emit_node = adj.node_visitors[type(node)]

        return emit_node(adj, node)

    # helper to evaluate expressions of the form
    # obj1.obj2.obj3.attr in the function's global scope
    def resolve_path(adj, path):
        if len(path) == 0:
            return None

        # if root is overshadowed by local symbols, bail out
        if path[0] in adj.symbols:
            return None

        if path[0] in __builtins__:
            return __builtins__[path[0]]

        # Look up the closure info and append it to adj.func.__globals__
        # in case you want to define a kernel inside a function and refer
        # to variables you've declared inside that function:
        extract_contents = (
            lambda contents: contents
            if isinstance(contents, warp.context.Function) or not callable(contents)
            else contents
        )
        capturedvars = dict(
            zip(
                adj.func.__code__.co_freevars,
                [extract_contents(c.cell_contents) for c in (adj.func.__closure__ or [])],
            )
        )
        vars_dict = {**adj.func.__globals__, **capturedvars}

        if path[0] in vars_dict:
            func = vars_dict[path[0]]

        # Support Warp types in kernels without the module suffix (e.g. v = vec3(0.0,0.2,0.4)):
        else:
            func = getattr(warp, path[0], None)

        if func:
            for i in range(1, len(path)):
                if hasattr(func, path[i]):
                    func = getattr(func, path[i])

        return func

    # Evaluates a static expression that does not depend on runtime values
    # if eval_types is True, try resolving the path using evaluated type information as well
    def resolve_static_expression(adj, root_node, eval_types=True):
        attributes = []

        node = root_node
        while isinstance(node, ast.Attribute):
            attributes.append(node.attr)
            node = node.value

        if eval_types and isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            # support for operators returning modules
            # i.e. operator_name(*operator_args).x.y.z
            operator_args = node.args
            operator_name = node.func.id

            if operator_name == "type":
                if len(operator_args) != 1:
                    raise WarpCodegenError(f"type() operator expects exactly one argument, got {len(operator_args)}")

                # type() operator
                var = adj.eval(operator_args[0])

                if isinstance(var, Var):
                    var_type = strip_reference(var.type)
                    # Allow accessing type attributes, for instance array.dtype
                    while attributes:
                        attr_name = attributes.pop()
                        var_type, prev_type = adj.resolve_type_attribute(var_type, attr_name), var_type

                        if var_type is None:
                            raise WarpCodegenAttributeError(
                                f"{attr_name} is not an attribute of {type_repr(prev_type)}"
                            )

                    return var_type, [type_repr(var_type)]
                else:
                    raise WarpCodegenError(f"Cannot deduce the type of {var}")

        # reverse list since ast presents it backward order
        path = [*reversed(attributes)]
        if isinstance(node, ast.Name):
            path.insert(0, node.id)

        # Try resolving path from captured context
        captured_obj = adj.resolve_path(path)
        if captured_obj is not None:
            return captured_obj, path

        # Still nothing found, maybe this is a predefined type attribute like `dtype`
        if eval_types:
            try:
                val = adj.eval(root_node)
                if val:
                    return [val, type_repr(val)]

            except Exception:
                pass

        return None, path

    # annotate generated code with the original source code line
    def set_lineno(adj, lineno):
        if adj.lineno is None or adj.lineno != lineno:
            line = lineno + adj.fun_lineno
            source = adj.source_lines[lineno].strip().ljust(80 - len(adj.indentation), " ")
            adj.add_forward(f"// {source}       <L {line}>")
            adj.add_reverse(f"// adj: {source}  <L {line}>")
        adj.lineno = lineno

    def get_node_source(adj, node):
        # return the Python code corresponding to the given AST node
        return ast.get_source_segment(adj.source, node)


# ----------------
# code generation

cpu_module_header = """
#define WP_NO_CRT
#include "builtin.h"

// avoid namespacing of float type for casting to float type, this is to avoid wp::float(x), which is not valid in C++
#define float(x) cast_float(x)
#define adj_float(x, adj_x, adj_ret) adj_cast_float(x, adj_x, adj_ret)

#define int(x) cast_int(x)
#define adj_int(x, adj_x, adj_ret) adj_cast_int(x, adj_x, adj_ret)

#define builtin_tid1d() wp::tid(wp::s_threadIdx)
#define builtin_tid2d(x, y) wp::tid(x, y, wp::s_threadIdx, dim)
#define builtin_tid3d(x, y, z) wp::tid(x, y, z, wp::s_threadIdx, dim)
#define builtin_tid4d(x, y, z, w) wp::tid(x, y, z, w, wp::s_threadIdx, dim)

"""

cuda_module_header = """
#define WP_NO_CRT
#include "builtin.h"

// avoid namespacing of float type for casting to float type, this is to avoid wp::float(x), which is not valid in C++
#define float(x) cast_float(x)
#define adj_float(x, adj_x, adj_ret) adj_cast_float(x, adj_x, adj_ret)

#define int(x) cast_int(x)
#define adj_int(x, adj_x, adj_ret) adj_cast_int(x, adj_x, adj_ret)

#define builtin_tid1d() wp::tid(_idx)
#define builtin_tid2d(x, y) wp::tid(x, y, _idx, dim)
#define builtin_tid3d(x, y, z) wp::tid(x, y, z, _idx, dim)
#define builtin_tid4d(x, y, z, w) wp::tid(x, y, z, w, _idx, dim)

"""

struct_template = """
struct {name}
{{
{struct_body}

    CUDA_CALLABLE {name}({forward_args})
    {forward_initializers}
    {{
    }}

    CUDA_CALLABLE {name}& operator += (const {name}& rhs)
    {{{prefix_add_body}
        return *this;}}

}};

static CUDA_CALLABLE void adj_{name}({reverse_args})
{{
{reverse_body}}}

CUDA_CALLABLE void adj_atomic_add({name}* p, {name} t)
{{
{atomic_add_body}}}


"""

cpu_forward_function_template = """
// {filename}:{lineno}
static {return_type} {name}(
    {forward_args})
{{
{forward_body}}}

"""

cpu_reverse_function_template = """
// {filename}:{lineno}
static void adj_{name}(
    {reverse_args})
{{
{reverse_body}}}

"""

cuda_forward_function_template = """
// {filename}:{lineno}
static CUDA_CALLABLE {return_type} {name}(
    {forward_args})
{{
{forward_body}}}

"""

cuda_reverse_function_template = """
// {filename}:{lineno}
static CUDA_CALLABLE void adj_{name}(
    {reverse_args})
{{
{reverse_body}}}

"""

cuda_kernel_template = """

extern "C" __global__ void {name}_cuda_kernel_forward(
    {forward_args})
{{
    for (size_t _idx = static_cast<size_t>(blockDim.x) * static_cast<size_t>(blockIdx.x) + static_cast<size_t>(threadIdx.x);
         _idx < dim.size;
         _idx += static_cast<size_t>(blockDim.x) * static_cast<size_t>(gridDim.x))
    {{
{forward_body}    }}
}}

extern "C" __global__ void {name}_cuda_kernel_backward(
    {reverse_args})
{{
    for (size_t _idx = static_cast<size_t>(blockDim.x) * static_cast<size_t>(blockIdx.x) + static_cast<size_t>(threadIdx.x);
         _idx < dim.size;
         _idx += static_cast<size_t>(blockDim.x) * static_cast<size_t>(gridDim.x))
    {{
{reverse_body}    }}
}}

"""

cpu_kernel_template = """

void {name}_cpu_kernel_forward(
    {forward_args})
{{
{forward_body}}}

void {name}_cpu_kernel_backward(
    {reverse_args})
{{
{reverse_body}}}

"""

cpu_module_template = """

extern "C" {{

// Python CPU entry points
WP_API void {name}_cpu_forward(
    {forward_args})
{{
    for (size_t i=0; i < dim.size; ++i)
    {{
        wp::s_threadIdx = i;

        {name}_cpu_kernel_forward(
            {forward_params});
    }}
}}

WP_API void {name}_cpu_backward(
    {reverse_args})
{{
    for (size_t i=0; i < dim.size; ++i)
    {{
        wp::s_threadIdx = i;

        {name}_cpu_kernel_backward(
            {reverse_params});
    }}
}}

}} // extern C

"""

cuda_module_header_template = """

extern "C" {{

// Python CUDA entry points
WP_API void {name}_cuda_forward(
    void* stream,
    {forward_args});

WP_API void {name}_cuda_backward(
    void* stream,
    {reverse_args});

}} // extern C
"""

cpu_module_header_template = """

extern "C" {{

// Python CPU entry points
WP_API void {name}_cpu_forward(
    {forward_args});

WP_API void {name}_cpu_backward(
    {reverse_args});

}} // extern C
"""


# converts a constant Python value to equivalent C-repr
def constant_str(value):
    value_type = type(value)

    if value_type == bool or value_type == builtins.bool:
        if value:
            return "true"
        else:
            return "false"

    elif value_type == str:
        # ensure constant strings are correctly escaped
        return '"' + str(value.encode("unicode-escape").decode()) + '"'

    elif isinstance(value, ctypes.Array):
        if value_type._wp_scalar_type_ == float16:
            # special case for float16, which is stored as uint16 in the ctypes.Array
            from warp.context import runtime

            scalar_value = runtime.core.half_bits_to_float
        else:

            def scalar_value(x):
                return x

        # list of scalar initializer values
        initlist = []
        for i in range(value._length_):
            x = ctypes.Array.__getitem__(value, i)
            initlist.append(str(scalar_value(x)))

        dtypestr = f"wp::initializer_array<{value._length_},wp::{value._wp_scalar_type_.__name__}>"

        # construct value from initializer array, e.g. wp::initializer_array<4,wp::float32>{1.0, 2.0, 3.0, 4.0}
        return f"{dtypestr}{{{', '.join(initlist)}}}"

    elif value_type in warp.types.scalar_types:
        # make sure we emit the value of objects, e.g. uint32
        return str(value.value)

    elif value == math.inf:
        return "INFINITY"

    else:
        # otherwise just convert constant to string
        return str(value)


def indent(args, stops=1):
    sep = ",\n"
    for i in range(stops):
        sep += "    "

    # return sep + args.replace(", ", "," + sep)
    return sep.join(args)


# generates a C function name based on the python function name
def make_full_qualified_name(func):
    if not isinstance(func, str):
        func = func.__qualname__
    return re.sub("[^0-9a-zA-Z_]+", "", func.replace(".", "__"))


def codegen_struct(struct, device="cpu", indent_size=4):
    name = make_full_qualified_name(struct.cls)

    body = []
    indent_block = " " * indent_size

    if len(struct.vars) > 0:
        for label, var in struct.vars.items():
            body.append(var.ctype() + " " + label + ";\n")
    else:
        # for empty structs, emit the dummy attribute to avoid any compiler-specific alignment issues
        body.append("char _dummy_;\n")

    forward_args = []
    reverse_args = []

    forward_initializers = []
    reverse_body = []
    atomic_add_body = []
    prefix_add_body = []

    # forward args
    for label, var in struct.vars.items():
        var_ctype = var.ctype()
        forward_args.append(f"{var_ctype} const& {label} = {{}}")
        reverse_args.append(f"{var_ctype} const&")

        namespace = "wp::" if var_ctype.startswith("wp::") or var_ctype == "bool" else ""
        atomic_add_body.append(f"{indent_block}{namespace}adj_atomic_add(&p->{label}, t.{label});\n")

        prefix = f"{indent_block}," if forward_initializers else ":"
        forward_initializers.append(f"{indent_block}{prefix} {label}{{{label}}}\n")

    # prefix-add operator
    for label, var in struct.vars.items():
        if not is_array(var.type):
            prefix_add_body.append(f"{indent_block}{label} += rhs.{label};\n")

    # reverse args
    for label, var in struct.vars.items():
        reverse_args.append(var.ctype() + " & adj_" + label)
        if is_array(var.type):
            reverse_body.append(f"{indent_block}adj_{label} = adj_ret.{label};\n")
        else:
            reverse_body.append(f"{indent_block}adj_{label} += adj_ret.{label};\n")

    reverse_args.append(name + " & adj_ret")

    return struct_template.format(
        name=name,
        struct_body="".join([indent_block + l for l in body]),
        forward_args=indent(forward_args),
        forward_initializers="".join(forward_initializers),
        reverse_args=indent(reverse_args),
        reverse_body="".join(reverse_body),
        prefix_add_body="".join(prefix_add_body),
        atomic_add_body="".join(atomic_add_body),
    )


def codegen_func_forward(adj, func_type="kernel", device="cpu"):
    if device == "cpu":
        indent = 4
    elif device == "cuda":
        if func_type == "kernel":
            indent = 8
        else:
            indent = 4
    else:
        raise ValueError(f"Device {device} not supported for codegen")

    indent_block = " " * indent

    # primal vars
    lines = []
    lines += ["//---------\n"]
    lines += ["// primal vars\n"]

    for var in adj.variables:
        if var.constant is None:
            lines += [f"{var.ctype()} {var.emit()};\n"]
        else:
            lines += [f"const {var.ctype()} {var.emit()} = {constant_str(var.constant)};\n"]

    # forward pass
    lines += ["//---------\n"]
    lines += ["// forward\n"]

    for f in adj.blocks[0].body_forward:
        lines += [f + "\n"]

    return "".join([indent_block + l for l in lines])


def codegen_func_reverse(adj, func_type="kernel", device="cpu"):
    if device == "cpu":
        indent = 4
    elif device == "cuda":
        if func_type == "kernel":
            indent = 8
        else:
            indent = 4
    else:
        raise ValueError(f"Device {device} not supported for codegen")

    indent_block = " " * indent

    lines = []

    # primal vars
    lines += ["//---------\n"]
    lines += ["// primal vars\n"]

    for var in adj.variables:
        if var.constant is None:
            lines += [f"{var.ctype()} {var.emit()};\n"]
        else:
            lines += [f"const {var.ctype()} {var.emit()} = {constant_str(var.constant)};\n"]

    # dual vars
    lines += ["//---------\n"]
    lines += ["// dual vars\n"]

    for var in adj.variables:
        lines += [f"{var.ctype(value_type=True)} {var.emit_adj()} = {{}};\n"]

    # forward pass
    lines += ["//---------\n"]
    lines += ["// forward\n"]

    for f in adj.blocks[0].body_replay:
        lines += [f + "\n"]

    # reverse pass
    lines += ["//---------\n"]
    lines += ["// reverse\n"]

    for l in reversed(adj.blocks[0].body_reverse):
        lines += [l + "\n"]

    # In grid-stride kernels the reverse body is in a for loop
    if device == "cuda" and func_type == "kernel":
        lines += ["continue;\n"]
    else:
        lines += ["return;\n"]

    return "".join([indent_block + l for l in lines])


def codegen_func(adj, c_func_name: str, device="cpu", options={}):
    # forward header
    if adj.return_var is not None and len(adj.return_var) == 1:
        return_type = adj.return_var[0].ctype()
    else:
        return_type = "void"

    has_multiple_outputs = adj.return_var is not None and len(adj.return_var) != 1

    forward_args = []
    reverse_args = []

    # forward args
    for i, arg in enumerate(adj.args):
        s = f"{arg.ctype()} {arg.emit()}"
        forward_args.append(s)
        if not adj.custom_reverse_mode or i < adj.custom_reverse_num_input_args:
            reverse_args.append(s)
    if has_multiple_outputs:
        for i, arg in enumerate(adj.return_var):
            forward_args.append(arg.ctype() + " & ret_" + str(i))
            reverse_args.append(arg.ctype() + " & ret_" + str(i))

    # reverse args
    for i, arg in enumerate(adj.args):
        if adj.custom_reverse_mode and i >= adj.custom_reverse_num_input_args:
            break
        # indexed array gradients are regular arrays
        if isinstance(arg.type, indexedarray):
            _arg = Var(arg.label, array(dtype=arg.type.dtype, ndim=arg.type.ndim))
            reverse_args.append(_arg.ctype() + " & adj_" + arg.label)
        else:
            reverse_args.append(arg.ctype() + " & adj_" + arg.label)
    if has_multiple_outputs:
        for i, arg in enumerate(adj.return_var):
            reverse_args.append(arg.ctype() + " & adj_ret_" + str(i))
    elif return_type != "void":
        reverse_args.append(return_type + " & adj_ret")
    # custom output reverse args (user-declared)
    if adj.custom_reverse_mode:
        for arg in adj.args[adj.custom_reverse_num_input_args :]:
            reverse_args.append(f"{arg.ctype()} & {arg.emit()}")

    if device == "cpu":
        forward_template = cpu_forward_function_template
        reverse_template = cpu_reverse_function_template
    elif device == "cuda":
        forward_template = cuda_forward_function_template
        reverse_template = cuda_reverse_function_template
    else:
        raise ValueError(f"Device {device} is not supported")

    # codegen body
    forward_body = codegen_func_forward(adj, func_type="function", device=device)

    s = ""
    if not adj.skip_forward_codegen:
        s += forward_template.format(
            name=c_func_name,
            return_type=return_type,
            forward_args=indent(forward_args),
            forward_body=forward_body,
            filename=adj.filename,
            lineno=adj.fun_lineno,
        )

    if not adj.skip_reverse_codegen:
        if adj.custom_reverse_mode:
            reverse_body = "\t// user-defined adjoint code\n" + forward_body
        else:
            if options.get("enable_backward", True):
                reverse_body = codegen_func_reverse(adj, func_type="function", device=device)
            else:
                reverse_body = '\t// reverse mode disabled (module option "enable_backward" is False)\n'
        s += reverse_template.format(
            name=c_func_name,
            return_type=return_type,
            reverse_args=indent(reverse_args),
            forward_body=forward_body,
            reverse_body=reverse_body,
            filename=adj.filename,
            lineno=adj.fun_lineno,
        )

    return s


def codegen_snippet(adj, name, snippet, adj_snippet):
    forward_args = []
    reverse_args = []

    # forward args
    for i, arg in enumerate(adj.args):
        s = f"{arg.ctype()} {arg.emit().replace('var_', '')}"
        forward_args.append(s)
        reverse_args.append(s)

    # reverse args
    for i, arg in enumerate(adj.args):
        if isinstance(arg.type, indexedarray):
            _arg = Var(arg.label, array(dtype=arg.type.dtype, ndim=arg.type.ndim))
            reverse_args.append(_arg.ctype() + " & adj_" + arg.label)
        else:
            reverse_args.append(arg.ctype() + " & adj_" + arg.label)

    forward_template = cuda_forward_function_template
    reverse_template = cuda_reverse_function_template

    s = ""
    s += forward_template.format(
        name=name,
        return_type="void",
        forward_args=indent(forward_args),
        forward_body=snippet,
        filename=adj.filename,
        lineno=adj.fun_lineno,
    )

    if adj_snippet:
        reverse_body = adj_snippet
    else:
        reverse_body = ""

    s += reverse_template.format(
        name=name,
        return_type="void",
        reverse_args=indent(reverse_args),
        forward_body=snippet,
        reverse_body=reverse_body,
        filename=adj.filename,
        lineno=adj.fun_lineno,
    )

    return s


def codegen_kernel(kernel, device, options):
    # Update the module's options with the ones defined on the kernel, if any.
    options = dict(options)
    options.update(kernel.options)

    adj = kernel.adj

    forward_args = ["wp::launch_bounds_t dim"]
    reverse_args = ["wp::launch_bounds_t dim"]

    # forward args
    for arg in adj.args:
        forward_args.append(arg.ctype() + " var_" + arg.label)
        reverse_args.append(arg.ctype() + " var_" + arg.label)

    # reverse args
    for arg in adj.args:
        # indexed array gradients are regular arrays
        if isinstance(arg.type, indexedarray):
            _arg = Var(arg.label, array(dtype=arg.type.dtype, ndim=arg.type.ndim))
            reverse_args.append(_arg.ctype() + " adj_" + arg.label)
        else:
            reverse_args.append(arg.ctype() + " adj_" + arg.label)

    # codegen body
    forward_body = codegen_func_forward(adj, func_type="kernel", device=device)

    if options["enable_backward"]:
        reverse_body = codegen_func_reverse(adj, func_type="kernel", device=device)
    else:
        reverse_body = ""

    if device == "cpu":
        template = cpu_kernel_template
    elif device == "cuda":
        template = cuda_kernel_template
    else:
        raise ValueError(f"Device {device} is not supported")

    s = template.format(
        name=kernel.get_mangled_name(),
        forward_args=indent(forward_args),
        reverse_args=indent(reverse_args),
        forward_body=forward_body,
        reverse_body=reverse_body,
    )

    return s


def codegen_module(kernel, device="cpu"):
    if device != "cpu":
        return ""

    adj = kernel.adj

    # build forward signature
    forward_args = ["wp::launch_bounds_t dim"]
    forward_params = ["dim"]

    for arg in adj.args:
        if hasattr(arg.type, "_wp_generic_type_str_"):
            # vectors and matrices are passed from Python by pointer
            forward_args.append(f"const {arg.ctype()}* var_" + arg.label)
            forward_params.append(f"*var_{arg.label}")
        else:
            forward_args.append(f"{arg.ctype()} var_{arg.label}")
            forward_params.append("var_" + arg.label)

    # build reverse signature
    reverse_args = [*forward_args]
    reverse_params = [*forward_params]

    for arg in adj.args:
        if isinstance(arg.type, indexedarray):
            # indexed array gradients are regular arrays
            _arg = Var(arg.label, array(dtype=arg.type.dtype, ndim=arg.type.ndim))
            reverse_args.append(f"const {_arg.ctype()} adj_{arg.label}")
            reverse_params.append(f"adj_{_arg.label}")
        elif hasattr(arg.type, "_wp_generic_type_str_"):
            # vectors and matrices are passed from Python by pointer
            reverse_args.append(f"const {arg.ctype()}* adj_{arg.label}")
            reverse_params.append(f"*adj_{arg.label}")
        else:
            reverse_args.append(f"{arg.ctype()} adj_{arg.label}")
            reverse_params.append(f"adj_{arg.label}")

    s = cpu_module_template.format(
        name=kernel.get_mangled_name(),
        forward_args=indent(forward_args),
        reverse_args=indent(reverse_args),
        forward_params=indent(forward_params, 3),
        reverse_params=indent(reverse_params, 3),
    )
    return s
