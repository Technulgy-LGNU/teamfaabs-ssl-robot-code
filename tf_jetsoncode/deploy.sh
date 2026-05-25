#!/usr/bin/env bash

set -e

TARGET="aarch64-unknown-linux-gnu"
BINARY="tf_jetsoncode"

REMOTE_DIR="/home/robotik/tf_jetsoncode"
REMOTE_BIN="$REMOTE_DIR"

ROBOTS=(
    "10.0.64.101"
)

MODE="debug"
SINGLE_ROBOT=""

usage() {
    echo "Usage:"
    echo "  ./deploy.sh all [debug|release]"
    echo "  ./deploy.sh robot <ip> [debug|release]"
    echo "  ./deploy.sh attach <ip>"
    exit 1
}

build() {
    echo "==> Building ($MODE)..."

    if [ "$MODE" = "release" ]; then
        /home/eb/.cargo/bin/cross build \
            --target $TARGET \
            --release

        LOCAL_BIN="target/$TARGET/release/$BINARY"
    else
        /home/eb/.cargo/bin/cross build \
            --target $TARGET

        LOCAL_BIN="target/$TARGET/debug/$BINARY"
    fi
}

deploy_robot() {
    local ROBOT_IP=$1

    echo ""
    echo "==> Deploying to $ROBOT_IP"

    ssh robotik@"$ROBOT_IP" "mkdir -p $REMOTE_DIR"


    scp "$LOCAL_BIN" \
        robotik@"$ROBOT_IP":$REMOTE_BIN

    ssh robotik@"$ROBOT_IP" \
        "chmod +x /home/robotik/tf_jetsoncode/tf_jetsoncode && sudo systemctl restart tf_jetsoncode"

    echo "==> Done: $ROBOT_IP"
}

attach_robot() {
    local ROBOT_IP=$1

    echo "==> Opening live logs for $ROBOT_IP"

    ssh -t robotik@"$ROBOT_IP" \
        "journalctl -fu tf_jetsoncode -o cat"
}

if [ $# -lt 1 ]; then
    usage
fi

COMMAND=$1

case $COMMAND in
    all)
        MODE=${2:-debug}

        build

        for ROBOT in "${ROBOTS[@]}"; do
            deploy_robot "$ROBOT"
        done
        ;;

    robot)
        SINGLE_ROBOT=$2
        MODE=${3:-debug}

        if [ -z "$SINGLE_ROBOT" ]; then
            usage
        fi

        build
        deploy_robot "$SINGLE_ROBOT"
        ;;

    attach)
        SINGLE_ROBOT=$2

        if [ -z "$SINGLE_ROBOT" ]; then
            usage
        fi

        attach_robot "$SINGLE_ROBOT"
        ;;

    *)
        usage
        ;;
esac