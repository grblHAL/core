// regex.h - A Regular Expression Matcher
//
// Code by Rob Pike, exegesis by Brian Kernighan
//
// http://genius.cat-v.org/brian-kernighan/articles/beautiful
//
//    c    matches any literal character c
//    .    matches any single character
//    ^    matches the beginning of the input string
//    $    matches the end of the input string
//    *    matches zero or more occurrences of the previous character

#pragma once

/* match: search for regexp anywhere in text */
int match(char *regexp, char *text);

/* matchhere: search for regexp at beginning of text */
int matchhere(char *regexp, char *text);

/* matchstar: search for c*regexp at beginning of text */
int matchstar(int c, char *regexp, char *text);
