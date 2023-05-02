MY_PATH=`dirname "$0"`
MY_PATH=`( cd "$MY_PATH" && pwd )`

ROOT_DIR="$MY_PATH/.."
BIN_DIR="$ROOT_DIR/cmake-build-debug/standalone/"

OUTPUT_FOLDER="$ROOT_DIR/results"

declare -a arr=(
    "wall_4_multi"
    "wall_8_multi"
    "wall_16_multi"
    "wall_32_multi"
    "wall_64_multi"
    "wall_128_multi"
    #"wall_256_multi"
    #"wall_512_multi"
    #"wall_1024_multi"
)

declare -a type=(
    "weakly"
    "highly"
)

declare -a type2=(
    "all"
    "part"
)

# wealky-all
# weakly-part
# highly-all
# highly-part

for j in "${type[@]}"
do
    for k in "${type2[@]}"
    do
        ## now loop through the above array
        for i in "${arr[@]}"
        do
            FNAME="${i}"
            PROBLEM_FILE="$MY_PATH/${FNAME}.txt"
            CONFIG_FILE="$ROOT_DIR/plans/${j}/planner_${j}_${FNAME}_${k}.yaml"
            touch "$PROBLEM_FILE"
            #pip install psrecord

            echo "running ${CONFIG_FILE}"

            BINARY="$BIN_DIR/optimal/planner_standalone_optimal"
            $BINARY \
            --name "${j}_${FNAME}_${k}" \
            --problem "$PROBLEM_FILE" \
            --config-file "$CONFIG_FILE" \
            --sop-solver lp \
            --output "$OUTPUT_FOLDER/optimal"

        done
    done
done

