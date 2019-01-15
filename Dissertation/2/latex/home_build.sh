#!/bin/bash

# Script to compile and place pdf file from tex and bib source files
# Modifiedto work in Windows subsystem for linux and copy files into the
# Documents directory in the Windows user directory.

pdflatex working
bibtex working
pdflatex working
pdflatex working

cp working.pdf ../pdf/diss.pdf
cp working.pdf '/mnt/c/Users/Robert Mitchell/Documents/diss.pdf'
