#!/usr/bin/perl -w

# Copyright: (C) 2011 RobotCub Consortium
# Author: Lorenzo Natale
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

my $fname = shift @ARGV;
my $line_num=0;
my $max_lines=60;
my %good_licenses= ( # a list of "good" licenses
		     "Released under the terms of the GNU GPL v2.0"=> "GPL20",
             "Permission is granted to copy, distribute, and/or modify this program under the terms of the GNU General Public License, version 2 or any later version published by the Free Software Foundation"=>"GPL20 or later",
			 "This program is free software; you can redistribute it and\/or modify it under the terms of the GNU General Public License version 2 as published by the Free Software Foundation"=>"GPL20",
             "Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met: 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer. 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution. 3. The name of the author may not be used to endorse or promote products derived from this software without specific prior written permission." => "BSD",
			 "BSD license" => "BSD",
             "Intel License Agreement For Open Source Computer Vision Library" => "OPENCV"
);


$fname=~s|\/\.\/|\/|;  # remove /./ pattern in directory names
$fname=~s|\/\/|\/|;    # remove // pattern in directory names

open(FILE, $fname) || 
    die "Can't open $fname";

my $author="unknown";
my $copyright="unknown";
my $license="unknown";

# Perform basic conversions to fix common problems
sub fix_names {
        my $names=$_[0];
        $names=~s/Alex Bernardino/Alexandre Bernardino/;
        $names=~s/\,?\s?European Commission FP6 Project IST(\-004370)?//;
        $names=~s/Robotcub/RobotCub/;
        $names=~s/^The\s|^\-//;

        $names=~s/\sall rights reserved//;

        $names=~s/\$YOUR_NAME/unknown/;

        return $names;
}

my $text="";

## collect file into a single variable, perform some pre-processing
while(!eof(FILE) && defined (my $line=<FILE>) && $line_num<$max_lines) {
    $line_num++;

    #skip emacs directives
    next if $line=~m/\-\*\-/;

    chomp $line;
    
	# remove comments 
    $line=~s/^\s*//;             #remove empty characters at beginning of line
    $line=~s/\s*$/ /;            #substitute trailing eof or spaces with single space
	$line =~s/^\#+//;            #remove comments like #
    $line =~s/^\/\*+//;          #remove /*
    $line =~s/\**\/*//;          #remove /* or ////
	$line =~s/^\*+//;            #remove * or ***
    $line=~s/^\s*//;             #now remove all empty characters at beginning of line

	
	$text .= $line;

    # now skip some tricky sentences that contain the word "author"
    # in licensing context
    next if $line=~m/The name of the author may not be used/i;
    next if $line=~m/BE LIABLE FOR ANY/i;
    next if $line=~m/\`*AS IS\'* AND ANY EXPRESS/i;
    
    # and word copyright
    next if $line=~m/copyright notice/i;

    if ($line=~m/authors?\s*(of changes)?\s*:?\s*/i && $author eq "unknown"){
        $author=$';
#       $author=~s/of changes\b*:?\b*//i; #remove pattern "of changes" (could not incoroprate it in prev. match
        $author=~s/\.?\s*$//;  #remove trailing . and eof ...

	# remove email addresses, reg exp from http://www.regular-expressions.info/email.html
        # adapted by adding spaces on top and optional quotes <>
	$author=~s/\s*<?\b[A-Z0-9._%+-]+@[A-Z0-9.-]+\.[A-Z]{2,4}\b>?//i;

	# remove html links
	$author=~s/\s*<A HREF.*?A>\.?//i;
    
        $author=fix_names($author);
    }

    if ($line=~m/copyright\s?(\s*:?\s*)?(\(C\))?\s*((<?\d*>?)|(\d*))(\,|\-?(\d*))*\s*\,?\s*(.*)$/i && $copyright eq "unknown"){
        $line=$8;

        # now knock all the trailing 2010-2010 etc..
        $line=~s/(\s*\,?\s*(\,?|\-?|\s?(\d*))*)\$//i;
        $line=~s/\.?\s*$//;  #remove trailing . and eof ...
        #print"--> $`<$&>$' $line\n";
        $copyright=$line;
        $copyright=fix_names($copyright);
    }

}

#print $text;

## Detect authors, copyright and licenses

# print "$line_num: $line\n";

#### search for known good licenses
while ( ($key, $value) = each %good_licenses) {
		
		if ($text=~m/$key/i) {
			#print "Match: $line_num: $line $value\n";
			$license=$value;
	}
}




print "File:$fname\n";
print "Copyright:$copyright\n";
print "Author:$author\n";
print "License:$license\n";
