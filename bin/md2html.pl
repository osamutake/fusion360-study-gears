use strict;
use warnings;
use utf8;

sub read_file {
  my $filename = shift;
  open(my $F, "<:utf8", $filename ) or die("error :$!");
  my $text = do { local $/; <$F> };
  close($F);
  return $text;
}

sub write_file {
  my ($filename, $data) = @_;
  open(my $F, ">:utf8", $filename) or die("error :$!");
  print $F $data;
  close($F);
}

foreach my $md (@ARGV) {
    my $base = $md;
    $base =~ s/\.md$//;
    system "markdown-to-html ".
           "--source $md --output $base.html ".
           "--title $base --no-dark-mode";

    my $appendToBody = << "EOS";
      <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/katex@0.16.21/dist/katex.min.css"
        integrity="sha384-zh0CIslj+VczCZtlzBcjt5ppRcsAmDnRem7ESsYwWwg3m/OaJ2l4x7YBZl9Kxxib"
        crossorigin="anonymous">
      <script defer src="https://cdn.jsdelivr.net/npm/katex@0.16.21/dist/katex.min.js"
        integrity="sha384-Rma6DA2IPUwhNxmrB/7S3Tno0YY7sFu9WSYMCuulLhIqYSGZ2gKCJWIqhBWqMQfh"
        crossorigin="anonymous"></script>
      <script defer src="https://cdn.jsdelivr.net/npm/katex@0.16.21/dist/contrib/auto-render.min.js"
        integrity="sha384-hCXGrW6PitJEwbkoStFjeJxv+fSOOQKOPbJxSfM6G5sWZjAyWhXiTIIAmQqnlLlh" 
        crossorigin="anonymous" onload="renderMathInElement(document.body, {
        delimiters: [{left: \'$$\', right: \'$$\', display: true}, {left: \'$\', right: \'$\', display: false}],
        throwOnError : false});"></script>
      <script>
        if (window.history.length == 1) {
          if (window.navigator.language.indexOf('ja') == 0) {
            window.location.href = 'README-ja.html';
          }
        }
      </script>
EOS

    my $html = read_file("$base.html");
    $html =~ s/<a href="([^"]+).md">/<a href="$1.html">/g;
    $html =~ s|"https://github.com/osamutake/fusion360-study-gears-docs/blob/master/|"docs/|g;
    $html =~ s|"https://github.com/osamutake/fusion360-study-gears/#|"../README.html#|g;
    $html =~ s|"https://github.com/osamutake/fusion360-study-gears/blob/main/README-ja.md#|"../README-ja.html#|g;
    $html =~ s|<\/body>|$appendToBody<\/body>|;

    write_file("$base.html", $html);
}
