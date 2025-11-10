#!/bin/bash
# Setup GPU virtual environment with CuRobo from source
# Minimal installation for collision checking only

set -e

# Get directories
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PCE_ROS_DIR="$(dirname "$SCRIPT_DIR")"
VENV_DIR="$PCE_ROS_DIR/venv"

echo "=========================================="
echo "Setting up GPU environment"
echo "=========================================="
echo "Package directory: $PCE_ROS_DIR"
echo "Virtual env: $VENV_DIR"
echo ""

# Check for NVIDIA GPU
if ! nvidia-smi &> /dev/null; then
    echo "ERROR: No NVIDIA GPU detected!"
    exit 1
fi

echo "✓ NVIDIA GPU detected:"
nvidia-smi --query-gpu=name --format=csv,noheader | head -n1

# Check CUDA
if ! nvcc --version &> /dev/null; then
    echo ""
    echo "ERROR: CUDA not found!"
    echo "Please install CUDA 11.8 or 12.x first."
    exit 1
fi

echo "✓ CUDA detected:"
nvcc --version | grep "release"
echo ""

# Create virtual environment if needed
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
    echo "✓ Virtual environment created"
fi

# Activate virtual environment
echo ""
echo "Activating virtual environment..."
source "$VENV_DIR/bin/activate"

if [[ "$VIRTUAL_ENV" != "$VENV_DIR" ]]; then
    echo "ERROR: Failed to activate virtual environment"
    exit 1
fi

echo "✓ Virtual environment activated"
echo "  Python: $(which python3)"
echo "  Python version: $(python3 --version)"

# Upgrade pip
echo ""
echo "Upgrading pip, setuptools, wheel..."
pip install --upgrade pip setuptools wheel

# Install PyTorch with CUDA
echo ""
echo "=========================================="
echo "Installing PyTorch with CUDA 11.8..."
echo "=========================================="
pip install torch==2.0.1 torchvision==0.15.2 --index-url https://download.pytorch.org/whl/cu118

# Verify PyTorch
python3 << 'EOF'
import torch
print(f"PyTorch version: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")
if not torch.cuda.is_available():
    print("ERROR: CUDA not available in PyTorch!")
    exit(1)
print(f"CUDA version: {torch.version.cuda}")
print(f"GPU: {torch.cuda.get_device_name(0)}")
EOF

if [ $? -ne 0 ]; then
    echo "ERROR: PyTorch installation failed!"
    exit 1
fi

# Install essential CuRobo dependencies only
echo ""
echo "=========================================="
echo "Installing CuRobo dependencies..."
echo "=========================================="
pip install ninja
pip install trimesh
pip install cython
pip install einops
pip install scipy
pip install matplotlib
pip install pyyaml
pip install typing-extensions

# Try to install warp-lang (optional, skip if fails)
echo ""
echo "Installing warp-lang (optional)..."
pip install warp-lang || echo "Warning: warp-lang installation failed (optional, continuing...)"

# Clone and install CuRobo from source
echo ""
echo "=========================================="
echo "Installing CuRobo from source..."
echo "=========================================="

CUROBO_DIR="$PCE_ROS_DIR/third_party/curobo"

if [ -d "$CUROBO_DIR" ]; then
    echo "CuRobo directory already exists."
    read -p "Update existing installation? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        cd "$CUROBO_DIR"
        git pull
    else
        cd "$CUROBO_DIR"
    fi
else
    echo "Cloning CuRobo repository..."
    mkdir -p "$PCE_ROS_DIR/third_party"
    cd "$PCE_ROS_DIR/third_party"
    git clone https://github.com/NVlabs/curobo.git
    cd curobo
fi

# Install CuRobo with minimal dependencies
echo "Installing CuRobo (collision checking only)..."

# Create a minimal setup that skips rendering dependencies
export CUROBO_MINIMAL_INSTALL=1

pip install -e . --no-deps
pip install -e .

# Install pybind11 for C++ integration
echo ""
echo "Installing pybind11..."
pip install pybind11

# Verify installation
echo ""
echo "=========================================="
echo "Verifying installation..."
echo "=========================================="
python3 << 'EOF'
import sys
import torch

print("="*60)
print("GPU Setup Verification")
print("="*60)
print(f"Python executable: {sys.executable}")
print(f"Python version: {sys.version}")
print(f"PyTorch version: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")

if not torch.cuda.is_available():
    print("\nERROR: CUDA not available in PyTorch!")
    sys.exit(1)

print(f"CUDA version: {torch.version.cuda}")
print(f"cuDNN version: {torch.backends.cudnn.version()}")
print(f"GPU device: {torch.cuda.get_device_name(0)}")
print(f"Number of GPUs: {torch.cuda.device_count()}")

# Test GPU computation
print("\nTesting GPU computation...")
x = torch.randn(1000, 1000, device='cuda')
y = x @ x.T
print("✓ GPU computation test passed")

# Test CuRobo imports (collision checking only)
print("\nTesting CuRobo imports...")
try:
    from curobo.types.base import TensorDeviceType
    print("✓ Imported TensorDeviceType")
    
    from curobo.types.robot import RobotConfig
    print("✓ Imported RobotConfig")
    
    from curobo.geom.types import WorldConfig
    print("✓ Imported WorldConfig")
    
    from curobo.wrap.reacher.motion_gen import MotionGenConfig
    print("✓ Imported MotionGenConfig")
    
    # Try to create basic config
    tensor_args = TensorDeviceType(device=torch.device("cuda:0"))
    print("✓ Created TensorDeviceType")
    
    print("\n✓ CuRobo core modules working!")
    
except ImportError as e:
    print(f"\nERROR: Failed to import CuRobo module: {e}")
    print("\nTrying to identify missing dependency...")
    import traceback
    traceback.print_exc()
    sys.exit(1)
except Exception as e:
    print(f"\nERROR: CuRobo initialization failed: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

print("\n" + "="*60)
print("✓ All GPU dependencies installed successfully!")
print("="*60)
EOF

if [ $? -ne 0 ]; then
    echo ""
    echo "ERROR: Installation verification failed!"
    echo ""
    echo "You can try installing missing dependencies manually:"
    echo "  source venv/bin/activate"
    echo "  pip install <missing_package>"
    exit 1
fi

# Create activation helper script
ACTIVATE_SCRIPT="$SCRIPT_DIR/activate_gpu.sh"
cat > "$ACTIVATE_SCRIPT" << 'ACTIVATESCRIPT'
#!/bin/bash
# Helper script to activate GPU environment

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PCE_ROS_DIR="$(dirname "$SCRIPT_DIR")"
VENV_DIR="$PCE_ROS_DIR/venv"

if [ ! -d "$VENV_DIR" ]; then
    echo "ERROR: Virtual environment not found at: $VENV_DIR"
    echo "Please run: ./scripts/setup_gpu_env.sh"
    return 1
fi

source "$VENV_DIR/bin/activate"
echo "✓ GPU environment activated: $VENV_DIR"
echo "  Python: $(which python3)"
ACTIVATESCRIPT

chmod +x "$ACTIVATE_SCRIPT"

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "Virtual environment: $VENV_DIR"
echo "CuRobo installed from: $CUROBO_DIR"
echo ""
echo "To activate in future sessions:"
echo "  source $PCE_ROS_DIR/scripts/activate_gpu.sh"
echo ""
echo "To build with GPU support:"
echo "  source $VENV_DIR/bin/activate"
echo "  cd ~/catkin_ws"
echo "  catkin build pce_ros -DUSE_GPU=ON"
echo ""