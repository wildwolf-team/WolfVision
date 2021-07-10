#!/bin/sh

if git rev-parse --verify HEAD > /dev/null 2>&1
then
    against=HEAD
else
    against=$(git rev-list --max-parents=0 HEAD)
fi

exec 1>&2

cpplint='python3 3rdparty/cpplint/cpplint.py'
sum=0
filters='-build/include_order,-build/namespaces,-legal/copyright,-runtime/references'
linelength=200

for file in $(git diff-index --name-status $against -- | grep -E '\.[ch](pp)?$' | awk '{print $2}' | awk '$0 !~ /3rdparty/'); do
    $cpplint --filter=$filters --linelength=$linelength $file
    sum=$(expr ${sum} + $?)
done

if [ ${sum} -eq 0 ]; then
    exit 0
else
    exit 1
fi
