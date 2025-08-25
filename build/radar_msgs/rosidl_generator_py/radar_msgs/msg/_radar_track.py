# generated from rosidl_generator_py/resource/_idl.py.em
# with input from radar_msgs:msg/RadarTrack.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RadarTrack(type):
    """Metaclass of message 'RadarTrack'."""

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
            module = import_type_support('radar_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'radar_msgs.msg.RadarTrack')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__radar_track
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__radar_track
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__radar_track
            cls._TYPE_SUPPORT = module.type_support_msg__msg__radar_track
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__radar_track

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RadarTrack(metaclass=Metaclass_RadarTrack):
    """Message class 'RadarTrack'."""

    __slots__ = [
        '_tracking_id',
        '_x_distance',
        '_y_distance',
        '_vx',
        '_vy',
    ]

    _fields_and_field_types = {
        'tracking_id': 'int32',
        'x_distance': 'float',
        'y_distance': 'float',
        'vx': 'float',
        'vy': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.tracking_id = kwargs.get('tracking_id', int())
        self.x_distance = kwargs.get('x_distance', float())
        self.y_distance = kwargs.get('y_distance', float())
        self.vx = kwargs.get('vx', float())
        self.vy = kwargs.get('vy', float())

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
        if self.tracking_id != other.tracking_id:
            return False
        if self.x_distance != other.x_distance:
            return False
        if self.y_distance != other.y_distance:
            return False
        if self.vx != other.vx:
            return False
        if self.vy != other.vy:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def tracking_id(self):
        """Message field 'tracking_id'."""
        return self._tracking_id

    @tracking_id.setter
    def tracking_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'tracking_id' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'tracking_id' field must be an integer in [-2147483648, 2147483647]"
        self._tracking_id = value

    @builtins.property
    def x_distance(self):
        """Message field 'x_distance'."""
        return self._x_distance

    @x_distance.setter
    def x_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'x_distance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x_distance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._x_distance = value

    @builtins.property
    def y_distance(self):
        """Message field 'y_distance'."""
        return self._y_distance

    @y_distance.setter
    def y_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'y_distance' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y_distance' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._y_distance = value

    @builtins.property
    def vx(self):
        """Message field 'vx'."""
        return self._vx

    @vx.setter
    def vx(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vx' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vx' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vx = value

    @builtins.property
    def vy(self):
        """Message field 'vy'."""
        return self._vy

    @vy.setter
    def vy(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vy' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vy' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vy = value
