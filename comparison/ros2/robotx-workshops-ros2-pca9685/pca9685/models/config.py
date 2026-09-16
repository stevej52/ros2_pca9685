from dataclasses import dataclass
from typing import Any, Dict
from pca9685.utils import is_hexadecimal

@dataclass
class PCA9685Config:
    busnum: int
    address: int
    frequency: int
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]):
        return cls(
            busnum=data["busnum"],
            address=cls.parse_address(data["address"]),
            frequency=data["frequency"],
        )
        
    @staticmethod
    def parse_address(address: Any) -> int:
        if isinstance(address, int):
            return address
        elif isinstance(address, str):
            if is_hexadecimal(address):
                try:
                    return int(address, 16)
                except ValueError:
                    raise ValueError(f"Invalid hexadecimal address: {address}")
            else:
                try:
                    return int(address)
                except ValueError:
                    raise ValueError(f"Invalid address format: {address}")
        else:
            raise TypeError(f"Address must be int or str, not {type(address)}")