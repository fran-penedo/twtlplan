PIPREQ=requirements.txt

LDIR=lib
TWTLDIR=$LDIR/twtl
LOMAPDIR=$TWTLDIR/src/lomap
ANTLR=antlr-3.1.3

TWTLURL=https://github.com/wasserfeder/twtl.git
LOMAPURL=https://github.com/wasserfeder/lomap.git
ANTLRURL=http://www.antlr3.org/download/$ANTLR.tar.gz

mkdir $LDIR

wget -O - $ANTLRURL | tar -xz -C $LDIR
cd $LDIR/$ANTLR/runtime/Python
python setup.py install
cd -

pip install -r $PIPREQ

git clone $TWTLURL $TWTLDIR
git clone $LOMAPURL $LOMAPDIR

cd $TWTLDIR
touch src/__init__.py
ant
cd -

