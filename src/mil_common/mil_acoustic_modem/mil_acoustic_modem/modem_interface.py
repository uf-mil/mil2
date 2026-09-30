from abc import ABC, abstractmethod


class ModemInterface(ABC):
    @abstractmethod
    def read_im():
        """Read a IM, timeout 2 seconds"""
        pass

    def send_im(message):
        """Send an IM"""
        pass
