#!/bin/sh
PWD=$(pwd)
for f in $(find ./positive -type f); do
	python2 ../test.py positive $PWD/$f
done

for f in $(find ./negative -type f); do
	python2 ../test.py negative $PWD/$f
done
