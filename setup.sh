#!/bin/bash

PIPREQ=requirements.txt

LDIR=lib
TWTLDIR=$LDIR/twtl
LOMAPDIR=$LDIR/lomap
ANTLR=antlr-3.1.3

TWTLURL=https://github.com/wasserfeder/twtl.git
LOMAPURL=https://github.com/wasserfeder/lomap.git
ANTLRURL=http://www.antlr3.org/download/$ANTLR.tar.gz

mkdir $LDIR

wget -O - $ANTLRURL | tar -xz -C $LDIR
cd $LDIR/$ANTLR/runtime/Python
python setup.py install
cd -

pip install cython
pip install -r $PIPREQ

git clone $TWTLURL $TWTLDIR
git clone $LOMAPURL $LOMAPDIR

cd $TWTLDIR
git checkout 7499d83
touch src/__init__.py
ant
mv src twtl
cd -

cd $LOMAPDIR
git checkout ec06a52
cd -
