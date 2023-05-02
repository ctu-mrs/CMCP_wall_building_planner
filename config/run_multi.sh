MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

ROOT_DIR="$MY_PATH/.."
BIN_DIR="$ROOT_DIR/cmake-build-debug/standalone/"

CONFIG_FILE="$ROOT_DIR/config/planner.yaml"
OUTPUT_FOLDER="$ROOT_DIR/results"

declare -a arr=(
    "wall_4_multi"
    "wall_8_multi"
    "wall_16_multi"
    "wall_32_multi"
    "wall_64_multi"
    "wall_128_multi"
    "wall_256_multi"
    "wall_512_multi"
    "wall_1024_multi"
)

## now loop through the above array
for i in "${arr[@]}"
do
    FNAME="${i}"
    PROBLEM_FILE="$MY_PATH/${FNAME}.txt"
    touch "$PROBLEM_FILE"
    #pip install psrecord

    BINARY="$BIN_DIR/grasp/planner_standalone_grasp_multi"
    $BINARY \
    --problem "$PROBLEM_FILE" \
    --config-file "$CONFIG_FILE" \
    --output "$OUTPUT_FOLDER/grasp"

done