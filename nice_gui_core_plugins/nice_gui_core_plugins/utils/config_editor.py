import yaml

from nicegui import ui


class ConfigEditor(ui.dialog):

    def __init__(self) -> None:
        """Config editor
        """
        super().__init__()

        with self, ui.card():
            self.content = ui.textarea(label="Edit yaml").props('full_width')
            ui.button('Save', on_click=self.on_save)

    def on_save(self):
        try:
            # Load the updated YAML content
            new_data = yaml.safe_load(self.content.value)
            with open(self._path, 'w') as file:
                yaml.dump(new_data, file)
            ui.notify('YAML saved successfully!')
        except yaml.YAMLError as e:
            ui.notify(f'Error saving YAML: {e}', color='red')
        self.close()

    def close(self) -> None:
        self.content.set_value("")
        self.value = False

    def open(self, path: str) -> None:
        """Open the dialog."""
        self.value = True
        self._path = path
        self.show_yaml(path)

    def show_yaml(self, path: str):
        with open(path) as file:
            data = yaml.safe_load(file)
            self.content.set_value(yaml.dump(data))
