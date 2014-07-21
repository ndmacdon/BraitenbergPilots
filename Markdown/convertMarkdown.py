"""

Filename:   convertMarkdown.py

Author:     Nicholas Macdonald

Project:    Braitenberg Pilots

Created:    July 20, 2014

Description:A simple Markdown converter built on Python-Markdown.
            Converts a file ('src.txt' by default) into an HTML file.

            Requires Python-Markdown: https://pythonhosted.org/Markdown/index.html

            Call with no arguments for the default behavior:

            $   python convertMarkdown.py
            >>  Reads: 'src.txt' Writes: 'dst.html'


            Call with 2 arguments to specify your own files:

            $   python convertMarkdown.py foo.txt bar.html
            >>  Reads: 'foo.txt' Writes 'bar.html'
"""

import markdown
import sys

def convertFile(source, destination):
  """
  Open the text-file named '<source>'' and write it
  to a file named '<destination>':
  """
  with open(destination, 'w') as f:
    markdown.markdownFromFile(
      input = source, 
      output = f,
      outputFormat = 'html5',
      extensions=['extra', 'headerid', 'sane_lists', 'smarty', 'toc(title=Index)']
      )

if __name__ == '__main__':
  if len(sys.argv) == 1:
    convertFile('src.txt', 'dst.html')

  elif len(sys.argv) == 3:
    if sys.argv[1] == sys.argv[2]:
      raise StandardError("The same file cannot be used as the source and destination file.")
    else:
      convertFile(sys.argv[1], sys.argv[2])

  else:
    raise TypeError("convertMarkdown() takes exactly 0 or 2 arguments (%d given)" % (len(sys.argv)-1))
