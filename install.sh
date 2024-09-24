LLM_BASE_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export LLM_BASE_PATH
cd $LLM_BASE_PATH

# Copy .usd to Isaac Sim
rm -rf ~/.local/share/ov/pkg/isaac-sim-2*/extscache/omni.importer.urdf*/data/urdf/robots/RIZON4
cd ~/.local/share/ov/pkg/isaac-sim-2*/extscache/omni.importer.urdf*/data/urdf/robots
mkdir RIZON4
cd $LLM_BASE_PATH
cp -r ./planning/rmpflow/flexiv_rizon4_total ~/.local/share/ov/pkg/isaac-sim-2*/extscache/omni.importer.urdf*/data/urdf/robots/RIZON4

# Copy planning and simulation files to Isaac Sim
rm -rf ~/.local/share/ov/pkg/isaac-sim-2*/standalone_examples/api/omni.isaac.manipulators/RIZON4
cd ~/.local/share/ov/pkg/isaac-sim-2*/standalone_examples/api/omni.isaac.manipulators
cd $LLM_BASE_PATH
cp -r ./planning ~/.local/share/ov/pkg/isaac-sim-2*/standalone_examples/api/omni.isaac.manipulators/
cd ~/.local/share/ov/pkg/isaac-sim-2*/standalone_examples/api/omni.isaac.manipulators/
mv planning RIZON4
