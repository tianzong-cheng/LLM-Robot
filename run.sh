LLM_BASE_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export LLM_BASE_PATH
cd $LLM_BASE_PATH

rm -rf temp
mkdir temp
mkdir temp/objects
mkdir temp/order
rm -rf ~/.local/share/ov/pkg/isaac-sim-2*/standalone_examples/api/omni.isaac.manipulators/RIZON4/simulate_datasets
cd ~/.local/share/ov/pkg/isaac-sim-2*/standalone_examples/api/omni.isaac.manipulators/RIZON4
mkdir simulate_datasets
cd $LLM_BASE_PATH

rm -rf debug
mkdir debug

# Go to home position
./flexiv/build/home 192.168.2.100 192.168.2.109

# Prompt the user for input
read -p "Input: " user_input
echo "$user_input" >temp/user_input.txt

python3 LLM/gdino_openai.py

echo -e "\e[33m[INFO] Running perception module ...\e[0m"
bash ./llm_robot_perception/run.sh

echo -e "\e[33m[INFO] Running coord_converter module ...\e[0m"
python3 ./coord_converter.py

echo -e "\e[33m[INFO] Running LLM module ...\e[0m"
python3 LLM/main_openai.py

echo -e "\e[33m[INFO] Running planning module ...\e[0m"
cp temp/planning_full.py ~/.local/share/ov/pkg/isaac-sim-2*/standalone_examples/api/omni.isaac.manipulators/RIZON4
cd ~/.local/share/ov/pkg/isaac-sim-2*
./python.sh standalone_examples/api/omni.isaac.manipulators/RIZON4/planning_full.py

cd $LLM_BASE_PATH
cp ~/.local/share/ov/pkg/isaac-sim-2*/standalone_examples/api/omni.isaac.manipulators/RIZON4/simulate_datasets/* ./temp

for file in ./temp/order*.txt; do
    # Check if the file exists
    if [[ -f "$file" ]]; then
        rm ./temp/order/*

        LLM_ORDER_NUMBER=$(basename "$file" | sed 's/order\([0-9]*\)\.txt/\1/')
        export LLM_ORDER_NUMBER

        python3 flexiv/data_converter.py

        ./flexiv/build/run 192.168.2.100 192.168.2.109
    fi
done
