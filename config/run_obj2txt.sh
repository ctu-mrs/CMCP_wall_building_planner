MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

ROOT_DIR="$MY_PATH/.."
BIN_DIR="$ROOT_DIR/cmake-build-debug/standalone/obj2txt"

BINARY="$BIN_DIR/planner_standalone_obj2txt"


CONFIG_FILE="$ROOT_DIR/config/planner.yaml"
OUTPUT_FOLDER="$ROOT_DIR/results"

declare -a arr=(
    #"wall_4_multi"
    "wall_8_multi"
    #"wall_16_multi"
    #"wall_32_multi"
    #"wall_64_multi"
    #"wall_128_multi"
    #"wall_256_multi"
    #"wall_512_multi"
    #"wall_1024_multi"
)

## now loop through the above array
for i in "${arr[@]}"
do
    PROBLEM_FILE="$MY_PATH/${i}.txt"
    PROBLEM_OBJ_FILE="$MY_PATH/${i}.obj"
    touch "$PROBLEM_FILE"
    #pip install psrecord
    $BINARY \
    --problem "$PROBLEM_FILE" \
    --problemobj "$PROBLEM_OBJ_FILE" \
    --sop-solver lp \
    --config-file "$CONFIG_FILE" \
    --output "$OUTPUT_FOLDER"
done