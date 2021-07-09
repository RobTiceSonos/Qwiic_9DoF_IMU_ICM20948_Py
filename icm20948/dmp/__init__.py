from . import fifo, images, regs, sensors


class Uninitialized(Exception):
    """ Raised when a DMP method is used prior to initialization """
    pass


class InitFailure(Exception):
    """ Raised when an error occurs in DMP initialization """
    pass


class FirmwareVerify(Exception):
    """ Raised when the verification of firmware fails during load """
    pass
