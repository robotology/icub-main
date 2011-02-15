#!/usr/bin/perl -w

# Copyright: (C) 2011 RobotCub Consortium
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

my $fname = shift @ARGV;
my %good_licenses= ( # a list of "good" licenses
		    "GPL20" => 1,
            "BSD" => 1);
my %authors=();

open GOOD, ">licenses-good.txt";
open BAD, ">licenses-bad.txt";
open AUTHORS, ">licenses-authors.txt";
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

    $author=~s/Author://;
    $license=~s/License://;
    $copyright=~s/Copyright://;

    ## clean year from authors/names
    $copyright=~s/\s*(\d\d\d\d)\s*//;
    $author=~s/\s*(\d\d\d\d)\s*//;
    
    $filename=~s/File://;

    if ($license=~m/unknown/i) {
         print BAD "$filename: missing license information\n";
    }
    else {
        if ($good_licenses{$license}) {
            print GOOD "$filename: $license\n";
        }
        else {
            print BAD "$filename: $license\n";
        }
    }

    if ($copyright=~m/unknown/i) {
        print BAD "$filename: missing copyright information\n";
    }
    else {
         my @authors=split('\s*\,\s*|\sand\s+', $copyright);

        foreach my $aut (@authors)
        {
            $authors{$aut}++;
        }
    }

    if ($author=~m/unknown/i) {
    }
    else {
        @names=split(',\s*|\sand\s+', $author);
        foreach $name (@names)
            { $authors{$name}++; }
    }
}

print AUTHORS "==== List of Authors ====\n";
while ( ($key, $value) = each %authors) 
{
    print AUTHORS "$key: $value\n";
}
    
close BAD;
close GOOD;
close AUTHORS;
