// Ref: https://docs.mathjax.org/en/latest/options/index.html
// Ref: https://squidfunk.github.io/mkdocs-material/setup/extensions/python-markdown-extensions/#arithmatex
// Ref: https://facelessuser.github.io/pymdown-extensions/extensions/arithmatex/#loading-mathjax
// Ref: https://docs.mathjax.org/en/latest/input/tex/extensions.html
window.MathJax = {
  // TODO (Customize LaTeX Extension):
  // - Refer to the link below:
  //       https://docs.mathjax.org/en/latest/input/tex/extensions.html
  tex: {
    inlineMath: [["$", "$"], ["\\(", "\\)"]],
    displayMath: [["$$", "$$"], ["\\[", "\\]"]],
    processEscapes: true,
    processEnvironments: true,
    packages: {'[+]': ['color']}
  },
  loader: {load: ['[tex]/color']},
  options: {
    // Ref: https://docs.mathjax.org/en/latest/options/document.html#option-descriptions
    // ignoreHtmlClass: ".*|",
    processHtmlClass: "arithmatex"
  }
};

// This portion may still be required if using Instant Loading.
// Ref: https://github.com/squidfunk/mkdocs-material/issues/2530
// document$.subscribe(() => {
//   MathJax.typesetPromise()
// })
