echo "LaTeX error recovery"
echo "Backing up working files"
mkdir backup
mv working.tex backup
mv working.pdf backup
mv working.bib backup

echo "Removing old log files"
rm working*

echo "Restoring working files"
cp backup/* .

echo "Remember to reload files in emacs"
