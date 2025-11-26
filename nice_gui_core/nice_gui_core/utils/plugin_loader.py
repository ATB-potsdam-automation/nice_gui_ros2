from importlib.metadata import entry_points


def load_plugins(group: str) -> dict:
    """
    Load all plugins registered under a given entry point group.

    Returns:
        Dict[str, Type] mapping entry point name -> plugin class
    """
    eps = entry_points(group=group)
    return {ep.name: ep.load() for ep in eps}  # name -> class
