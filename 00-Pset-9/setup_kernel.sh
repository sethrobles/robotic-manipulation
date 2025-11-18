#!/bin/bash
# Setup script to create .venv and register it as a Jupyter kernel

# Create virtual environment if it doesn't exist
if [ ! -d ".venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv .venv
fi

# Activate virtual environment
source .venv/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install requirements
if [ -f "requirements.txt" ]; then
    echo "Installing requirements..."
    pip install -r requirements.txt
fi

# Install ipykernel to register as Jupyter kernel
echo "Installing ipykernel..."
pip install ipykernel

# Register the kernel with Jupyter
echo "Registering kernel..."
python -m ipykernel install --user --name=pset9-venv --display-name="Python (pset9-venv)"

echo "Kernel setup complete!"
echo "In your notebook, select the kernel: 'Python (pset9-venv)'"
