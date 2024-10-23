---
tags: [demo]
dlbook_notations: true
---

# DL Book Notations

This document provides the notation used in the [Deep Learning Book](https://www.deeplearningbook.org/), and is mostly based on this [LaTex file](https://github.com/goodfeli/dlbook_notation/blob/master/notation.tex). You can refer to the source of this post for the latex commands.

## Numbers and Arrays

| Notation | Description |
|----------|-------------|
| $\displaystyle a$ | A scalar (integer or real) |
| $\displaystyle \va$ | A vector |
| $\displaystyle \mA$ | A matrix |
| $\displaystyle \tA$ | A tensor |
| $\displaystyle \mI_n$ | Identity matrix with $n$ rows and $n$ columns |
| $\displaystyle \mI$ | Identity matrix with dimensionality implied by context |
| $\displaystyle \ve^{(i)}$ | Standard basis vector $[0,\dots,0,1,0,\dots,0]$ with a 1 at position $i$ |
| $\displaystyle \text{diag}(\va)$ | A square, diagonal matrix with diagonal entries given by $\va$ |
| $\displaystyle \ra$ | A scalar random variable |
| $\displaystyle \rva$ | A vector-valued random variable |
| $\displaystyle \rmA$ | A matrix-valued random variable |

## Sets and Graphs

| Notation | Description |
|----------|-------------|
| $\displaystyle \sA$ | A set |
| $\displaystyle \R$ | The set of real numbers |
| $\displaystyle \{0, 1\}$ | The set containing 0 and 1 |
| $\displaystyle \{0, 1, \dots, n \}$ | The set of all integers between $0$ and $n$ |
| $\displaystyle [a, b]$ | The real interval including $a$ and $b$ |
| $\displaystyle (a, b]$ | The real interval excluding $a$ but including $b$ |
| $\displaystyle \sA \backslash \sB$ | Set subtraction, i.e., the set containing the elements of $\sA$ that are not in $\sB$ |
| $\displaystyle \gG$ | A graph |
| $\displaystyle \parents_\gG(\ervx_i)$ | The parents of $\ervx_i$ in $\gG$ |

## Indexing

| Notation | Description |
|----------|-------------|
| $\displaystyle \eva_i$ | Element $i$ of vector $\va$, with indexing starting at 1 |
| $\displaystyle \eva_{-i}$ | All elements of vector $\va$ except for element $i$ |
| $\displaystyle \emA_{i,j}$ | Element $i, j$ of matrix $\mA$ |
| $\displaystyle \mA_{i, :}$ | Row $i$ of matrix $\mA$ |
| $\displaystyle \mA_{:, i}$ | Column $i$ of matrix $\mA$ |
| $\displaystyle \etA_{i, j, k}$ | Element $(i, j, k)$ of a 3-D tensor $\tA$ |
| $\displaystyle \tA_{:, :, i}$ | 2-D slice of a 3-D tensor |
| $\displaystyle \erva_i$ | Element $i$ of the random vector $\rva$ |

## Linear Algebra Operations

| Notation | Description |
|----------|-------------|
| $\displaystyle \mA^\top$ | Transpose of matrix $\mA$ |
| $\displaystyle \mA^+$ | Moore-Penrose pseudoinverse of $\mA$ |
| $\displaystyle \mA \odot \mB$ | Element-wise (Hadamard) product of $\mA$ and $\mB$ |
| $\displaystyle \mathrm{det}(\mA)$ | Determinant of $\mA$ |

## Calculus

| Notation | Description |
|----------|-------------|
| $\displaystyle\frac{d y} {d x}$ | Derivative of $y$ with respect to $x$ |
| $\displaystyle \frac{\partial y} {\partial x}$ | Partial derivative of $y$ with respect to $x$ |
| $\displaystyle \nabla_\vx y$ | Gradient of $y$ with respect to $\vx$ |
| $\displaystyle \nabla_\mX y$ | Matrix derivatives of $y$ with respect to $\mX$ |
| $\displaystyle \nabla_\tX y$ | Tensor containing derivatives of $y$ with respect to $\tX$ |
| $\displaystyle \frac{\partial f}{\partial \vx}$ | Jacobian matrix $\mJ \in \R^{m\times n}$ of $f: \R^n \rightarrow \R^m$ |
| $\displaystyle \nabla_\vx^2 f(\vx)\text{ or }\mH( f)(\vx)$ | The Hessian matrix of $f$ at input point $\vx$ |
| $\displaystyle \int f(\vx) d\vx$ | Definite integral over the entire domain of $\vx$ |
| $\displaystyle \int_\sS f(\vx) d\vx$ | Definite integral with respect to $\vx$ over the set $\sS$ |

## Probability and Information Theory

| Notation | Description |
|----------|-------------|
| $\displaystyle \ra \bot \rb$ | The random variables $\ra$ and $\rb$ are independent |
| $\displaystyle \ra \bot \rb \mid \rc$ | They are conditionally independent given $\rc$ |
| $\displaystyle P(\ra)$ | A probability distribution over a discrete variable |
| $\displaystyle p(\ra)$ | A probability distribution over a continuous variable, or over a variable whose type has not been specified |
| $\displaystyle \ra \sim P$ | Random variable $\ra$ has distribution $P$ |
| $\displaystyle  \E_{\rx\sim P} [ f(x) ]\text{ or } \E f(x)$ | Expectation of $f(x)$ with respect to $P(\rx)$ |
| $\displaystyle \Var(f(x))$ |  Variance of $f(x)$ under $P(\rx)$  |
| $\displaystyle \Cov(f(x),g(x))$ | Covariance of $f(x)$ and $g(x)$ under $P(\rx)$ |
| $\displaystyle H(\rx)$ | Shannon entropy of the random variable $\rx$ |
| $\displaystyle \KL ( P \Vert Q )$ | Kullback-Leibler divergence of P and Q |
| $\displaystyle \mathcal{N} ( \vx ; \vmu , \mSigma)$ | Gaussian distribution over $\vx$ with mean $\vmu$ and covariance $\mSigma$ |

## Functions

| Notation | Description |
|----------|-------------|
| $\displaystyle f: \sA \rightarrow \sB$ | The function $f$ with domain $\sA$ and range $\sB$ |
| $\displaystyle f \circ g$ | Composition of the functions $f$ and $g$ |
| $\displaystyle f(\vx ; \vtheta)$ | A function of $\vx$ parametrized by $\vtheta$. (Sometimes we write $f(\vx)$ and omit the argument $\vtheta$ to lighten notation) |
| $\displaystyle \log x$ | Natural logarithm of $x$ |
| $\displaystyle \sigma(x)$ | Logistic sigmoid, $\displaystyle \frac{1} {1 + \exp(-x)}$ |
| $\displaystyle \zeta(x)$ | Softplus, $\log(1 + \exp(x))$ |
| $\displaystyle \vert\vert \vx \vert\vert_p$ | $\normlp$ norm of $\vx$ |
| $\displaystyle \vert\vert \vx \vert\vert$ | $\normltwo$ norm of $\vx$ |
| $\displaystyle x^+$ | Positive part of $x$, i.e., $\max(0,x)$ |
| $\displaystyle \1_\mathrm{condition}$ | is 1 if the condition is true, 0 otherwise |

Sometimes we use a function $f$ whose argument is a scalar but apply
it to a vector, matrix, or tensor: $f(\vx)$, $f(\mX)$, or $f(\tX)$.
This denotes the application of $f$ to the
array element-wise. For example, if $\tC = \sigma(\tX)$, then $\etC_{i,j,k} = \sigma(\etX_{i,j,k})$
for all valid values of $i$, $j$ and $k$.

## Datasets and Distributions

| Notation | Description |
|----------|-------------|
| $\displaystyle \pdata$ | The data generating distribution |
| $\displaystyle \ptrain$ | The empirical distribution defined by the training set |
| $\displaystyle \sX$ | A set of training examples |
| $\displaystyle \vx^{(i)}$ | The $i$-th example (input) from a dataset |
| $\displaystyle y^{(i)}\text{ or }\vy^{(i)}$ | The target associated with $\vx^{(i)}$ for supervised learning |
| $\displaystyle \mX$ | The $m \times n$ matrix with input example $\vx^{(i)}$ in row $\mX_{i,:}$ |

## Final Notes

For using these commands, your post should have the following yaml config in the beginning of your markdown file:

```yaml
---
dlbook_notations: true
---
```
