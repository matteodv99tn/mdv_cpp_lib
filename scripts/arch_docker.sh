DOCKER_IMAGE="archlinux:base"
CONTAINER_NAME="arch_mdv_cpp"

if  [[ $1 ]]; then case "$1" in
    -h|--help|help)
        echo "Starts a docker container names '$CONTAINER_NAME' based on image '$DOCKER_IMAGE'"
        echo "Possible parameters:"
        echo "  --pull:   pulls the image from docker hub"
        echo "  --stop:   stops the container (if running)"
        echo "  --rm:     removes the container"
        exit 0
        ;;
    --pull)
        echo "Pulling image $DOCKER_IMAGE"
        docker pull $DOCKER_IMAGE
        exit 0
        ;;
    --stop)
        echo "Stopping container $CONTAINER_NAME"
        docker stop $CONTAINER_NAME >/dev/null 2> /dev/null 
        exit 0
        ;;
    --rm)
        echo "Removing container $CONTAINER_NAME"
        docker stop $CONTAINER_NAME >/dev/null 2> /dev/null 
        docker rm $CONTAINER_NAME >/dev/null 2> /dev/null 
        exit 0
        ;;
esac
fi

# Get the absolute path of this script
# Source: https://stackoverflow.com/questions/59895
THIS_SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
ROOT_DIR=$(realpath $THIS_SCRIPT_DIR/..)

# Ensure that image is available
if [[ -z  "$(docker images -q $DOCKER_IMAGE 2> /dev/null)"  ]]; then
    echo "Image $DOCKER_IMAGE not found"
    echo "Make sure you have the image available before running this script"
    exit 1
fi

# Check if container is already running
if [[ -n "$(docker ps -q -f name=$CONTAINER_NAME)" ]]; then
    echo "Container $CONTAINER_NAME is already running, attaching to it.."
    docker exec -it $CONTAINER_NAME /bin/bash
else
    # Check if container is stopped
    if [[ -n "$(docker ps -aq -f status=exited -f name=$CONTAINER_NAME)" ]]; then
        echo "Container $CONTAINER_NAME is stopped, starting it.."
        docker start $CONTAINER_NAME
        docker exec -it $CONTAINER_NAME /bin/bash
    else
        echo "Container $CONTAINER_NAME not found, creating and starting it.."
        docker run \
            -it \
            -v $ROOT_DIR:/home \
            --net=host \
            --ipc=host \
            --shm-size 8g \
            --cpus=6 \
            --name $CONTAINER_NAME \
            $DOCKER_IMAGE /bin/bash
    fi
fi
