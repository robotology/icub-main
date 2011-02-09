#!/usr/bin/perl -w

# Copyright: (C) 2011 RobotCub Consortium
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

my $fname = shift @ARGV;

open(FILE, $fname) || 
    die "Can't open $fname";

## First we detect authors, copyright and licenses
while(!eof(FILE) && defined (my $line=<FILE>)) {

    next unless $line=~m/File/;    
    chomp $line;
    my $filename=$line;
    $line=<FILE>;
    chomp $line;
    my $copyright=$line;
    $line=<FILE>;
    chomp $line;
    my $author=$line;
    $line=<FILE>;
    chomp $line;
    $license=$line;

    $filename=~s/File://;
    if ($license=~m/License:unknown/i)
    {
	print "$filename: missing license information\n";
    }
    if ($copyright=~m/Copyright:unknown/i)
    {
	print "$filename: missing copyright information\n";
    }
}

    
