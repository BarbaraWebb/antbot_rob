#!/bin/bash

# Script to compile and place pdf file from tex and bib source files

pdflatex working
bibtex working
pdflatex working
pdflatex working

cp working.pdf ../pdf/diss.pdf
