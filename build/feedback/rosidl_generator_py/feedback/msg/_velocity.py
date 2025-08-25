# generated from rosidl_generator_py/resource/_idl.py.em
# with input from feedback:msg/Velocity.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Velocity(type):
    """Metaclass of message 'Velocity'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('feedback')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'feedback.msg.Velocity')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__velocity
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__velocity
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__velocity
            cls._TYPE_SUPPORT = module.type_support_msg__msg__velocity
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__velocity

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Velocity(metaclass=Metaclass_Velocity):
    """Message class 'Velocity'."""

    __slots__ = [
        '_vehicle_velocity',
        '_wheelrpm_fl',
        '_wheelrpm_fr',
        '_wheelrpm_rl',
        '_wheelrpm_rr',
    ]

    _fields_and_field_types = {
        'vehicle_velocity': 'float',
        'wheelrpm_fl': 'float',
        'wheelrpm_fr': 'float',
        'wheelrpm_rl': 'float',
        'wheelrpm_rr': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.vehicle_velocity = kwargs.get('vehicle_velocity', float())
        self.wheelrpm_fl = kwargs.get('wheelrpm_fl', float())
        self.wheelrpm_fr = kwargs.get('wheelrpm_fr', float())
        self.wheelrpm_rl = kwargs.get('wheelrpm_rl', float())
        self.wheelrpm_rr = kwargs.get('wheelrpm_rr', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.vehicle_velocity != other.vehicle_velocity:
            return False
        if self.wheelrpm_fl != other.wheelrpm_fl:
            return False
        if self.wheelrpm_fr != other.wheelrpm_fr:
            return False
        if self.wheelrpm_rl != other.wheelrpm_rl:
            return False
        if self.wheelrpm_rr != other.wheelrpm_rr:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def vehicle_velocity(self):
        """Message field 'vehicle_velocity'."""
        return self._vehicle_velocity

    @vehicle_velocity.setter
    def vehicle_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vehicle_velocity' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vehicle_velocity' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vehicle_velocity = value

    @builtins.property
    def wheelrpm_fl(self):
        """Message field 'wheelrpm_fl'."""
        return self._wheelrpm_fl

    @wheelrpm_fl.setter
    def wheelrpm_fl(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'wheelrpm_fl' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'wheelrpm_fl' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._wheelrpm_fl = value

    @builtins.property
    def wheelrpm_fr(self):
        """Message field 'wheelrpm_fr'."""
        return self._wheelrpm_fr

    @wheelrpm_fr.setter
    def wheelrpm_fr(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'wheelrpm_fr' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'wheelrpm_fr' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._wheelrpm_fr = value

    @builtins.property
    def wheelrpm_rl(self):
        """Message field 'wheelrpm_rl'."""
        return self._wheelrpm_rl

    @wheelrpm_rl.setter
    def wheelrpm_rl(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'wheelrpm_rl' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'wheelrpm_rl' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._wheelrpm_rl = value

    @builtins.property
    def wheelrpm_rr(self):
        """Message field 'wheelrpm_rr'."""
        return self._wheelrpm_rr

    @wheelrpm_rr.setter
    def wheelrpm_rr(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'wheelrpm_rr' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'wheelrpm_rr' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._wheelrpm_rr = value
