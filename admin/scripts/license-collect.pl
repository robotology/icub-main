#!/usr/bin/perl -w

# Copyright: (C) 2011 RobotCub Consortium
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

my $fname = shift @ARGV;
my $line_num=0;
my $max_lines=3;

$fname=~s|\/\.\/|\/|;

open(FILE, $fname) || 
    die "Can't open $fname";

my $author="unknown";
my $copyright="unknown";
my $license="unknown";

## Detect authors, copyright and licenses
while(!eof(FILE) && defined (my $line=<FILE>) || $line_num<$max_lines) {
    $line_num++;

    next unless $line=~m/Copyright|Author|Copypolicy/i;
    chomp $line;
#    print "$line_num: $line\n";
    if ($line=~m/copypolicy:?\s*/i){
	$license=$';
    }
    if ($line=~m/authors?:?\s*/i){
	$author=$';
    }
    if ($line=~m/copyright\s?(\s*:?\s*)?(\(C\))?\s?(\d\d\d\d)?\s*/i){
	$copyright=$'.' '.$3;
    }
}

print "File:$fname\n";
print "Copyright:$copyright\n";
print "Author:$author\n";
print "License:$license\n";

    
