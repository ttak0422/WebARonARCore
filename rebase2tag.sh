# Check that at least the tag name has been provided.
if [ $# -ne 1 ]; then
	echo "ERROR: No tag specified. Usage: ./rebase2tag.sh 57.0.2987.5"
	exit 1
fi
# Get the name of the current branch
BRANCH_NAME=$(git symbolic-ref -q HEAD)
BRANCH_NAME=${BRANCH_NAME##refs/heads/}
BRANCH_NAME=${BRANCH_NAME:-HEAD}
# Create a new branch from the current one with the new name webar_TAG
git checkout -b "webar_$1"
if [ $? -ne 0 ]; then exit 1; fi
mkdir "out/webar_$1"
if [ $? -ne 0 ]; then exit 1; fi
cp "out/$BRANCH_NAME/args.gn" "out/webar_$1"
if [ $? -ne 0 ]; then exit 1; fi
# Create a new branch from the tag
git checkout -b "$1" "$1"
if [ $? -ne 0 ]; then exit 1; fi
git pull
if [ $? -ne 0 ]; then exit 1; fi
# Checkout the branch again
git checkout "webar_$1"
if [ $? -ne 0 ]; then exit 1; fi
# Rebase
git rebase $1
