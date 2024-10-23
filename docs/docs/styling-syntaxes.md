---
tags: [demo]
---

# Styling Syntaxes

In this document, we'll showcase some commonly used features that MkDocs can achieve in a generated website. All of them can work out-of-the-box if you forked the [**MkDocs Material Template**](https://github.com/j3soon/mkdocs-material-template).

## Markdown Basics

See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) on GitHub Guides for the basic usages.

The remainder of this post consists of pairs of generated content followed by its corresponding syntax.

<hr>

Normal text

```
Normal text
```

<hr>

Newline
Failed

Newline  
Success (Note the trailing spaces above)

Newline

Large Gap

```md
Newline
Failed

Newline  
Success (Note the trailing spaces above)

Newline

Large Gap
```

<hr>

> Snippet from other sites

```
> Snippet from other sites
```

<hr>

`inline-code`

```
`inline-code`
```

<hr>

**Bold**

```
**Bold**
```

<hr>

```
code
```

````
```
code
```
````

<hr>

```c
int a = 0; // c code
```

````
```c
int a = 0; // c code
```
````

<hr>

```js
console.log("test"); // js code
```

````
```js
int a = 0; // js code
```
````

<hr>

1. one
   - a  
     Contents should be indented correctly
   - b  
     2 trailing spaces to ensure line-break
   - c

     Extra line for large gap

2. two
   - c
     Line-break failed
   - d  
     Line-break success

   No Indent

```md
1. one
   - a  
     Contents should be indented correctly
   - b  
     2 trailing spaces to ensure line-break
   - c

     Extra line for large gap

2. two
   - c
     Line-break failed
   - d  
     Line-break success

   No Indent
```

<hr>

|Table|Left|Center|Right|
|-----|----|:----:|----:|
|A|B|C|D|
|1|2|3|4|

```md
|Table|Left|Center|Right|
|-----|----|:----:|----:|
|A|B|C|D|
|1|2|3|4|
```

<hr>

![alt-text-of-sample-image]({{imgroot}}{{page.url}}/img.png)

{% raw %}
```md
![alt-text-of-sample-image]({{imgroot}}{{page.url}}/img.png)
```
{% endraw %}

<hr>

[In-line link (look bad in plain text)](https://google.com)

[Named link][google]

[google]: https://google.com

```
[In-line link (look bad in plain text)](https://google.com)

[Named link][google]

[google]: https://google.com
```

<hr>

### Header example

#### Header example2

```md
### Header example

#### Header example2
```

## MathJax & LaTex

Inline math $f(x)=x^2$

```md
Inline math $f(x)=x^2$
```

<hr>

Long math

$\max\limits_\theta L_{\theta_0}(\theta)$, subject to $D_{KL}^{\rho_{\theta_0}}(\theta_0,\theta)\le\delta$, where $D_{KL}^\rho(\theta_1,\theta_2)=\mathbb{E}_{s\sim\rho}[D_{KL}(\pi_{\theta_1}(\cdot | s)\mid\mid\pi_{\theta_2}(\cdot | s))]$

```latex
$\max\limits_\theta L_{\theta_0}(\theta)$, subject to $D_{KL}^{\rho_{\theta_0}}(\theta_0,\theta)\le\delta$, where $D_{KL}^\rho(\theta_1,\theta_2)=\mathbb{E}_{s\sim\rho}[D_{KL}(\pi_{\theta_1}(\cdot | s)\mid\mid\pi_{\theta_2}(\cdot | s))]$
```

<hr>

Centered math:

$$f(x)=x^2$$

```md
$$f(x)=x^2$$
```

<hr>

|Symbols|Using|Latex|
|:-:|:-:|:-:|
|$✔$|$\color{green} ✔$|$\color{red} ✘$|

```latex
|Symbols|Using|Latex|
|:-:|:-:|:-:|
|$✔$|$\color{green} ✔$|$\color{red} ✘$|
```

<hr>

$$\begin{aligned}
G_t&=R_{t+1}+\gamma R_{t+2}+\gamma^2 R_{t+3}+\gamma^3 R_{t+4}+...\\
&=R_{t+1}+\gamma(R_{t+2}+\gamma R_{t+3}+\gamma^2 R_{t+4})+...\\
&=R_{t+1}+\gamma G_{t+1}\\
\end{aligned}$$

```latex
$$\begin{aligned}
G_t&=R_{t+1}+\gamma R_{t+2}+\gamma^2 R_{t+3}+\gamma^3 R_{t+4}+...\\
&=R_{t+1}+\gamma(R_{t+2}+\gamma R_{t+3}+\gamma^2 R_{t+4})+...\\
&=R_{t+1}+\gamma G_{t+1}\\
\end{aligned}$$
```

<hr>

## YAML

Tags should be listed in the beginning of the file with YAML syntax.

```yaml
---
tags: [demo]
---
```

<hr>

## Liquid

{{page}}

{% raw %}
```liquid
{{page}}
```
{% endraw %}

<hr>

## Material for MkDocs

### Admonitions

!!! note

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.

```
!!! note

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.
```

<hr>

!!! abstract

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.

```
!!! abstract

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.
```

<hr>

!!! info

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.

```
!!! info

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.
```

<hr>

!!! tip

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.

```
!!! tip

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.
```

<hr>

!!! success

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.

```
!!! success

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.
```

<hr>

!!! question

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.

```
!!! question

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.
```

<hr>

!!! warning

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.

```
!!! warning

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.
```

<hr>

!!! failure

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.

```
!!! failure

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.
```

<hr>

!!! danger

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.

```
!!! danger

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.
```

<hr>

!!! bug

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.

```
!!! bug

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.
```

<hr>

!!! example

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.

```
!!! example

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.
```

<hr>

!!! quote

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.

```
!!! quote

    Lorem ipsum dolor sit amet, consectetur adipiscing elit. Nulla et
    euismod nulla. Curabitur feugiat, tortor non consequat finibus, justo
    purus auctor massa, nec semper lorem quam in massa.
```

<hr>

See [this link](https://squidfunk.github.io/mkdocs-material/reference/admonitions/) for more info.

### Icons & Emojis

:smile:

```
:smile:
```

See [this link](https://squidfunk.github.io/mkdocs-material/reference/icons-emojis) for more info.

## Final Notes

There may exist notation conflicts between Markdown and MathJax. In rare cases, you may need to use the escaped text (e.g., `\vert`, `\_`) instead of the original symbol (e.g., `|`, `_`).
