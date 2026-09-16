def is_hexadecimal(value: str) -> bool:
    try:
        int(value, 16)
        return value.lower().startswith("0x")
    except ValueError:
        return False
