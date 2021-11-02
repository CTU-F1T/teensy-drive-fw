#!/usr/bin/env bash

set -e

# see https://unix.stackexchange.com/questions/155046/determine-if-git-working-directory-is-clean-from-a-script
is_git_clean() {
	if [ -z "$(git status --porcelain)" ]; then
		return 0
	else
		return 1
	fi
}

if ! is_git_clean; then
	echo "git working directory must be clean (including no untracked files)"
	echo "please commit or stash any changes before updating teensy3 core"
	exit 1
fi

rm -rf temp
mkdir -p temp
cd temp
git clone https://github.com/PaulStoffregen/cores
cd cores/teensy3
git describe --always
teensy3_version=$(git describe --dirty --always)
teensy3_revision=$(git rev-parse HEAD)
cat <<-EOF >README.txt
	This is copy of the teensy3 dir from the https://github.com/PaulStoffregen/cores
	Version: $teensy3_version
	Commit: https://github.com/PaulStoffregen/cores/commit/$teensy3_revision
EOF
rm main.cpp
cd ../../..
rm -rf teensy3
cp -r temp/cores/teensy3/ teensy3/
rm -rf temp

git add teensy3
git commit -m "Update teensy3 core to PaulStoffregen/cores@$teensy3_revision"

echo "teensy3 core updated to PaulStoffregen/cores@$teensy3_revision"
