from google.protobuf import descriptor_pb2 as _descriptor_pb2
from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class FieldType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    FT_DEFAULT: _ClassVar[FieldType]
    FT_CALLBACK: _ClassVar[FieldType]
    FT_POINTER: _ClassVar[FieldType]
    FT_STATIC: _ClassVar[FieldType]
    FT_IGNORE: _ClassVar[FieldType]
    FT_INLINE: _ClassVar[FieldType]

class IntSize(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    IS_DEFAULT: _ClassVar[IntSize]
    IS_8: _ClassVar[IntSize]
    IS_16: _ClassVar[IntSize]
    IS_32: _ClassVar[IntSize]
    IS_64: _ClassVar[IntSize]

class TypenameMangling(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    M_NONE: _ClassVar[TypenameMangling]
    M_STRIP_PACKAGE: _ClassVar[TypenameMangling]
    M_FLATTEN: _ClassVar[TypenameMangling]
    M_PACKAGE_INITIALS: _ClassVar[TypenameMangling]

class DescriptorSize(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    DS_AUTO: _ClassVar[DescriptorSize]
    DS_1: _ClassVar[DescriptorSize]
    DS_2: _ClassVar[DescriptorSize]
    DS_4: _ClassVar[DescriptorSize]
    DS_8: _ClassVar[DescriptorSize]
FT_DEFAULT: FieldType
FT_CALLBACK: FieldType
FT_POINTER: FieldType
FT_STATIC: FieldType
FT_IGNORE: FieldType
FT_INLINE: FieldType
IS_DEFAULT: IntSize
IS_8: IntSize
IS_16: IntSize
IS_32: IntSize
IS_64: IntSize
M_NONE: TypenameMangling
M_STRIP_PACKAGE: TypenameMangling
M_FLATTEN: TypenameMangling
M_PACKAGE_INITIALS: TypenameMangling
DS_AUTO: DescriptorSize
DS_1: DescriptorSize
DS_2: DescriptorSize
DS_4: DescriptorSize
DS_8: DescriptorSize
NANOPB_FILEOPT_FIELD_NUMBER: _ClassVar[int]
nanopb_fileopt: _descriptor.FieldDescriptor
NANOPB_MSGOPT_FIELD_NUMBER: _ClassVar[int]
nanopb_msgopt: _descriptor.FieldDescriptor
NANOPB_ENUMOPT_FIELD_NUMBER: _ClassVar[int]
nanopb_enumopt: _descriptor.FieldDescriptor
NANOPB_FIELD_NUMBER: _ClassVar[int]
nanopb: _descriptor.FieldDescriptor

class NanoPBOptions(_message.Message):
    __slots__ = ("max_size", "max_length", "max_count", "int_size", "enum_intsize", "type", "long_names", "packed_struct", "packed_enum", "skip_message", "no_unions", "msgid", "anonymous_oneof", "proto3", "proto3_singular_msgs", "enum_to_string", "enum_validate", "fixed_length", "fixed_count", "submsg_callback", "mangle_names", "callback_datatype", "callback_function", "descriptorsize", "default_has", "include", "exclude", "package", "type_override", "label_override", "sort_by_tag", "fallback_type", "initializer", "discard_unused_automatic_types", "discard_deprecated")
    MAX_SIZE_FIELD_NUMBER: _ClassVar[int]
    MAX_LENGTH_FIELD_NUMBER: _ClassVar[int]
    MAX_COUNT_FIELD_NUMBER: _ClassVar[int]
    INT_SIZE_FIELD_NUMBER: _ClassVar[int]
    ENUM_INTSIZE_FIELD_NUMBER: _ClassVar[int]
    TYPE_FIELD_NUMBER: _ClassVar[int]
    LONG_NAMES_FIELD_NUMBER: _ClassVar[int]
    PACKED_STRUCT_FIELD_NUMBER: _ClassVar[int]
    PACKED_ENUM_FIELD_NUMBER: _ClassVar[int]
    SKIP_MESSAGE_FIELD_NUMBER: _ClassVar[int]
    NO_UNIONS_FIELD_NUMBER: _ClassVar[int]
    MSGID_FIELD_NUMBER: _ClassVar[int]
    ANONYMOUS_ONEOF_FIELD_NUMBER: _ClassVar[int]
    PROTO3_FIELD_NUMBER: _ClassVar[int]
    PROTO3_SINGULAR_MSGS_FIELD_NUMBER: _ClassVar[int]
    ENUM_TO_STRING_FIELD_NUMBER: _ClassVar[int]
    ENUM_VALIDATE_FIELD_NUMBER: _ClassVar[int]
    FIXED_LENGTH_FIELD_NUMBER: _ClassVar[int]
    FIXED_COUNT_FIELD_NUMBER: _ClassVar[int]
    SUBMSG_CALLBACK_FIELD_NUMBER: _ClassVar[int]
    MANGLE_NAMES_FIELD_NUMBER: _ClassVar[int]
    CALLBACK_DATATYPE_FIELD_NUMBER: _ClassVar[int]
    CALLBACK_FUNCTION_FIELD_NUMBER: _ClassVar[int]
    DESCRIPTORSIZE_FIELD_NUMBER: _ClassVar[int]
    DEFAULT_HAS_FIELD_NUMBER: _ClassVar[int]
    INCLUDE_FIELD_NUMBER: _ClassVar[int]
    EXCLUDE_FIELD_NUMBER: _ClassVar[int]
    PACKAGE_FIELD_NUMBER: _ClassVar[int]
    TYPE_OVERRIDE_FIELD_NUMBER: _ClassVar[int]
    LABEL_OVERRIDE_FIELD_NUMBER: _ClassVar[int]
    SORT_BY_TAG_FIELD_NUMBER: _ClassVar[int]
    FALLBACK_TYPE_FIELD_NUMBER: _ClassVar[int]
    INITIALIZER_FIELD_NUMBER: _ClassVar[int]
    DISCARD_UNUSED_AUTOMATIC_TYPES_FIELD_NUMBER: _ClassVar[int]
    DISCARD_DEPRECATED_FIELD_NUMBER: _ClassVar[int]
    max_size: int
    max_length: int
    max_count: int
    int_size: IntSize
    enum_intsize: IntSize
    type: FieldType
    long_names: bool
    packed_struct: bool
    packed_enum: bool
    skip_message: bool
    no_unions: bool
    msgid: int
    anonymous_oneof: bool
    proto3: bool
    proto3_singular_msgs: bool
    enum_to_string: bool
    enum_validate: bool
    fixed_length: bool
    fixed_count: bool
    submsg_callback: bool
    mangle_names: TypenameMangling
    callback_datatype: str
    callback_function: str
    descriptorsize: DescriptorSize
    default_has: bool
    include: _containers.RepeatedScalarFieldContainer[str]
    exclude: _containers.RepeatedScalarFieldContainer[str]
    package: str
    type_override: _descriptor_pb2.FieldDescriptorProto.Type
    label_override: _descriptor_pb2.FieldDescriptorProto.Label
    sort_by_tag: bool
    fallback_type: FieldType
    initializer: str
    discard_unused_automatic_types: bool
    discard_deprecated: bool
    def __init__(self, max_size: _Optional[int] = ..., max_length: _Optional[int] = ..., max_count: _Optional[int] = ..., int_size: _Optional[_Union[IntSize, str]] = ..., enum_intsize: _Optional[_Union[IntSize, str]] = ..., type: _Optional[_Union[FieldType, str]] = ..., long_names: bool = ..., packed_struct: bool = ..., packed_enum: bool = ..., skip_message: bool = ..., no_unions: bool = ..., msgid: _Optional[int] = ..., anonymous_oneof: bool = ..., proto3: bool = ..., proto3_singular_msgs: bool = ..., enum_to_string: bool = ..., enum_validate: bool = ..., fixed_length: bool = ..., fixed_count: bool = ..., submsg_callback: bool = ..., mangle_names: _Optional[_Union[TypenameMangling, str]] = ..., callback_datatype: _Optional[str] = ..., callback_function: _Optional[str] = ..., descriptorsize: _Optional[_Union[DescriptorSize, str]] = ..., default_has: bool = ..., include: _Optional[_Iterable[str]] = ..., exclude: _Optional[_Iterable[str]] = ..., package: _Optional[str] = ..., type_override: _Optional[_Union[_descriptor_pb2.FieldDescriptorProto.Type, str]] = ..., label_override: _Optional[_Union[_descriptor_pb2.FieldDescriptorProto.Label, str]] = ..., sort_by_tag: bool = ..., fallback_type: _Optional[_Union[FieldType, str]] = ..., initializer: _Optional[str] = ..., discard_unused_automatic_types: bool = ..., discard_deprecated: bool = ...) -> None: ...
