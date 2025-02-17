SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
echo "Dir: $SCRIPT_DIR"
echo "Compare: $HOME/mil2/scripts"

if [[ "$SCRIPT_DIR" != "$HOME/mil2/scripts" && -z ${ALLOW_NONSTANDARD_DIR:-}  ]]; then
  echo "The directories are different."
else
  echo "The directories are the same."
fi
