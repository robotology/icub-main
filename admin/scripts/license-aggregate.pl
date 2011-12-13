#!/usr/bin/perl -w

# Copyright: (C) 2011 RobotCub Consortium
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

my $fname = shift @ARGV;
my %good_licenses= ( # a list of "good" licenses
            "GPL20" => 1,
            "GPL20 or later" => 1,
            "BSD" => 1,
            "OPENCV"=> 1);

my %authors=();
my %copyright_owners=();

open GOOD, ">licenses-good.txt";
open BAD, ">licenses-bad.txt";
open AUTHORS, ">licenses-authors.txt";
open AUTHORS_COMPACT, ">licenses-authors-compact.txt";
open COPYRIGHT_COMPACT, ">licenses-copyright-compact.txt";

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
         my @names=split('\s*\,\s*|\sand\s+', $copyright);

        foreach my $name (@names)
        {
            $copyright_owners{$name}++;
        }
    }

    if ($author=~m/unknown/i) {
    }
    else {
        my @names=split(',\s*|\sand\s+', $author);
        foreach my $name (@names)
            { $authors{$name}++; }
    }
}

#print in sorted 

print AUTHORS "=== List of Copyright Owners ===\n";
foreach $value (sort {$copyright_owners{$b} <=> $copyright_owners{$a} } keys %copyright_owners)
{
    print AUTHORS "$value: $copyright_owners{$value}\n";
    print COPYRIGHT_COMPACT "* $value ($copyright_owners{$value} files)\n";
}

print AUTHORS "==== List of Authors ====\n";
foreach $value (sort {$authors{$b} <=> $authors{$a} } keys %authors)
{ 
    print AUTHORS "$value: $authors{$value}\n";
    print AUTHORS_COMPACT "* $value ($authors{$value} files)\n";
}



close BAD;
close GOOD;
close AUTHORS;
