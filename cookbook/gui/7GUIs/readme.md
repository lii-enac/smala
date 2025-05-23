This is the smala version of the 7GUIs GUI Programming Benchmark https://eugenkiss.github.io/7guis/

TODO:
- assess with the Cognitive Dimensions https://eugenkiss.github.io/7guis/dimensions
- assess by counting the number of lines that do not contain "// [7GUIs]" as a measure of boilerplate/scaffolding
  - without empty, comment-only and closing brace-only "}" lines 

find cookbook/gui/7GUIs/1-counter -name "*.sma" | xargs grep -v "\/\/ \[7GUIs\]" | grep -v "^\s*$" | grep -v "^\s*//" | grep -v "^\s*\}\s*" | wc -l

