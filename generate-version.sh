#!/usr/bin/env bash

# NOTE: `echo -n` in Makefile on macOS behaves incorrectly in some cases
#       see https://stackoverflow.com/questions/11675070/makefile-echo-n-not-working

echo "$@"

echo -n "#define VERSION \"s" >src/version.h
git log --oneline src | wc -l | xargs echo -n >>src/version.h
echo -n ".$1 / " >> src/version.h
git describe --dirty --always | xargs echo -n >> src/version.h
echo -n "\"" >> src/version.h
