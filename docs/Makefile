HTML_DIR = html
HTML_ENTRYPOINT = $(HTML_DIR)/index.html

MD_FILES = $(shell find . -name '*.md')
RST_FILES = $(shell find . -name '*.rst')
FILES = $(MD_FILES) $(RST_FILES)


# Python variables (for the virtual environment)
VENV = .venv
PYTHON = $(VENV)/bin/python3
PIP = $(VENV)/bin/pip
ACTIVATE = $(if $(wildcard $(VENV)/bin/activate), . $(VENV)/bin/activate, echo "Not using virtual environment")


.PHONY: docs view venv clean

docs: $(HTML_DIR)

clean:
	@echo "Removing files"
	@rm $(HTML_DIR) -r 2>>/dev/null || true

view: $(HTML_DIR)
	@echo Opening documentation
	xdg-open $(HTML_ENTRYPOINT) > /dev/null 2>&1 & disown	

# Shortcut for the virtual environment
venv: $(VENV)/bin/activate

$(HTML_DIR): $(FILES)
	@echo Building documentation
	@ $(ACTIVATE) && sphinx-build . $(HTML_DIR)

# Virtual environment
$(VENV)/bin/activate: requirements.txt
	@echo "Creating a new virtual environment..."
	@python3 -m venv $(VENV)
	@echo "Installing dependencies..."
	@. $(VENV)/bin/activate && python3 -m pip install -r requirements.txt
	@touch $(VENV)/bin/activate
	@echo "Dependencies installed."


