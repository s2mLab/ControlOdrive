from enum import Enum


class StrEnum(str, Enum):
    """
    Enum where members are also (and must be) strings. Existing class in Python 3.11 but the code is running in Python
    3.9 for now.
    """

    def __new__(cls, *values):
        """
        Values must already be of type `str`.
        """
        if len(values) > 3:
            raise TypeError('too many arguments for str(): %r' % (values,))
        if len(values) == 1:
            # it must be a string
            if not isinstance(values[0], str):
                raise TypeError('%r is not a string' % (values[0],))
        if len(values) >= 2:
            # check that encoding argument is a string
            if not isinstance(values[1], str):
                raise TypeError('encoding must be a string, not %r' % (values[1],))
        if len(values) == 3:
            # check that errors argument is a string
            if not isinstance(values[2], str):
                raise TypeError('errors must be a string, not %r' % (values[2]))
        value = str(*values)
        member = str.__new__(cls, value)
        member._value_ = value
        return member

    def _generate_next_value_(self, start, count, last_values):
        """
        Return the lower-cased version of the member name.
        """
        return self.lower()


def traduce_error(decimal_number, odrive_enum):
    hex_number = "{0:x}".format(decimal_number)
    res = ""
    for i, j in enumerate(hex_number[::-1]):
        if j != '0':
            for member in odrive_enum.__members__.values():
                if member.value == "0x" + format(int(j) * 16 ** int(i), f"0{8}X"):
                    res += member.name
                    res += ", "
    return res
