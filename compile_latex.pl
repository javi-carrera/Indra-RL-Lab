#!/usr/bin/perl
use strict;
use warnings;
use File::Basename;

# Get the LaTeX file name from command line arguments
my $latex_file = shift @ARGV or die "Usage: perl compile_latex.pl <filename.tex>\n";

# Check if the file exists
unless (-e $latex_file) {
    die "File '$latex_file' does not exist.\n";
}

# Get the basename of the file (without extension)
my $base_name = basename($latex_file, ".tex");

# Compile the LaTeX file using pdflatex
my $command = "pdflatex $latex_file";

# Execute the command
my $result = system($command);

# Check for errors during compilation
if ($result != 0) {
    die "Failed to compile '$latex_file' with pdflatex.\n";
}

# Run bibtex if needed (optional)
my $bibtex_command = "bibtex $base_name";
system($bibtex_command) == 0 or warn "BibTeX failed for '$base_name'.\n";

# Compile again to ensure references are updated
system($command) == 0 or die "Failed to compile '$latex_file' with pdflatex.\n";

# Optional: Cleanup auxiliary files
unlink glob("$base_name.*") or warn "Could not delete auxiliary files for '$base_name'.\n";

print "Successfully compiled '$latex_file' to '$base_name.pdf'.\n";
