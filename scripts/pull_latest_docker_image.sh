#!/bin/bash -e

IMAGE="${1:-ubuntu:22.04}"

get_local_image_digest() {
    docker image inspect "$IMAGE" --format '{{index .RepoDigests 0}}' 2>/dev/null | sed 's/.*@//'
}

get_image_repo() {
    local image_without_tag="${IMAGE%%:*}"
    if [[ "$image_without_tag" != */* ]]; then
        echo "library/${image_without_tag}"
    else
        echo "$image_without_tag"
    fi
}

get_image_tag() {
    if [[ "$IMAGE" == *:* ]]; then
        echo "${IMAGE##*:}"
    else
        echo "latest"
    fi
}

get_remote_image_digest() {
    local architecture
    local repo
    local tag

    case "$(uname -m)" in
        x86_64)
            architecture="amd64"
            ;;
        aarch64|arm64)
            architecture="arm64"
            ;;
        *)
            echo ""
            return 0
            ;;
    esac

    repo="$(get_image_repo)"
    tag="$(get_image_tag)"

    curl -fsSL "https://registry.hub.docker.com/v2/repositories/${repo}/tags/${tag}" | \
        jq -r --arg arch "$architecture" '
            .digest // (
                .images[] | select(.architecture == $arch and .os == "linux") | .digest
            )
        ' | \
        head -n 1
}

if ! command -v docker >/dev/null 2>&1; then
    echo "Docker is not available. Skipping ${IMAGE} update check."
    exit 0
fi

local_digest="$(get_local_image_digest)"
if [ -z "$local_digest" ]; then
    echo "Local ${IMAGE} image not found. Pulling it now."
    docker pull "$IMAGE"
    exit 0
fi

remote_digest="$(get_remote_image_digest)"
if [ -z "$remote_digest" ]; then
    echo "Unable to determine whether ${IMAGE} has a newer remote image. Skipping pull."
    exit 0
fi

if [ "$local_digest" = "$remote_digest" ]; then
    echo "${IMAGE} is already up to date."
    exit 0
fi

echo "A newer ${IMAGE} image is available."
echo "Local digest:  ${local_digest}"
echo "Remote digest: ${remote_digest}"

if [ -t 0 ]; then
    echo "Pull the newer ${IMAGE} image now? [y/N]"
    read -r answer
    case "$answer" in
        [yY]|[yY][eE][sS])
            docker pull "$IMAGE"
            ;;
        *)
            echo "Skipped pulling ${IMAGE}."
            ;;
    esac
else
    echo "Non-interactive mode detected. Skipping ${IMAGE} pull."
fi
