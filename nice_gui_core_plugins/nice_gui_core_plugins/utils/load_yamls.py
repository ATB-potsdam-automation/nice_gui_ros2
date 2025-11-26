import yaml


def load_yaml(yaml_file_path):
    """
    Loads a YAML file from the specified file path.

    Parameters:
    yaml_file_path (str): The path to the YAML file to be loaded.

    Returns:
    dict: Parsed data from the YAML file if loaded successfully; None otherwise.

    Raises:
    Exception: If an error occurs while loading the YAML file, it is re-raised after logging.
    """
    try:
        with open(yaml_file_path, 'r') as file:
            data = yaml.safe_load(file)
        return data
    except Exception as e:
        print(f"An error occurred while loading the YAML file: {e}")
        raise e
    return None
