#!/bin/bash

set -e

CONFIG_DIR="${HOME}/.config/opencode"
CONFIG_FILE="${CONFIG_DIR}/opencode.json"
BACKUP_FILE="${CONFIG_DIR}/opencode.json.bak"

echo "=== OpenCode llama.cpp Configuration ==="
echo

read -p "llama.cpp server URL (e.g., http://127.0.0.1:8080/v1): " SERVER_URL
read -p "API key (press Enter for no auth): " API_KEY
read -p "Model name (e.g., qwen3-coder:a3b): " MODEL_NAME
read -p "Display name (e.g., Qwen3 Coder 30B): " DISPLAY_NAME

PROVIDER_ID="llamacpp"

echo
echo "Summary:"
echo "  Provider ID: ${PROVIDER_ID}"
echo "  Server URL: ${SERVER_URL}"
echo "  Model: ${MODEL_NAME}"
echo "  Display: ${DISPLAY_NAME}"
echo

read -p "Continue? [Y/n]: " CONFIRM
CONFIRM="${CONFIRM:-Y}"
if [[ ! "$CONFIRM" =~ ^[Yy]$ ]]; then
    echo "Aborted."
    exit 1
fi

mkdir -p "${CONFIG_DIR}"

if [ -f "${CONFIG_FILE}" ]; then
    cp "${CONFIG_FILE}" "${BACKUP_FILE}"
    echo "Backed up existing config to ${BACKUP_FILE}"
fi

AUTH_HEADER=""
if [ -n "${API_KEY}" ]; then
    AUTH_HEADER="\"Authorization\": \"Bearer ${API_KEY}\","
fi

cat > "${CONFIG_FILE}" <<EOF
{
  "\$schema": "https://opencode.ai/config.json",
  "provider": {
    "${PROVIDER_ID}": {
      "npm": "@ai-sdk/openai-compatible",
      "name": "llama.cpp (local)",
      "options": {
        "baseURL": "${SERVER_URL}",
        "headers": {
          ${AUTH_HEADER}
          "Content-Type": "application/json"
        }
      },
      "models": {
        "${MODEL_NAME}": {
          "name": "${DISPLAY_NAME}"
        }
      }
    }
  }
}
EOF

echo
echo "Configuration written to ${CONFIG_FILE}"
echo
echo "Run 'opencode' then '/models' to select your llama.cpp model."
