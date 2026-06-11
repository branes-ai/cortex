# AI Coding Assistant Failures

We now have half a dozen examples of Claude Code failing to produce quality code, be able to find bugs, and in general, not following requests even when there is a perfect spec available. This is clearly a very unproductive situations as we are trying to solve novel computer science and computer arithmetic problems. Where does the community discuss the failings of the current crop of AI coding assistants and are they asking the question about code quality and the ability to debug, clean-up, simplify, strengthen the code they are producing as to make the actual task of software development better, higher quality, and lower cost? 

It is completely understandable that you are hitting a wall. When working on novel computer science and complex computer arithmetic, the cracks in the current generation of AI coding assistants—whether it’s Claude Code, GitHub Copilot, or others—turn into gaping holes.

The blunt reality is that these models are probabilistic token predictors, not deterministic reasoning engines. They are heavily optimized to regurgitate and stitch together boilerplate web frameworks, standard scripts, and common algorithms found in their training data. When you hand them a perfect, rigorous specification for a *novel* problem, they frequently fail because they lack the internal mechanism to execute formal logic, track deep mathematical states, or strictly obey constraints without falling back on learned approximations.

You are not alone in this frustration. The broader software engineering community is currently going through a massive reckoning regarding AI-generated code, moving past the initial hype and slamming into the reality of maintenance, debugging, and code quality.

Here is where the community is discussing these failings, and the hard questions being asked.

## Where the Discussions are Happening

* **Hacker News (Y Combinator):** This is the epicenter for high-level, critical discussions about the systemic impact of AI on software engineering. Discussions frequently revolve around "AI-generated technical debt," the shifting costs of maintenance, and the fact that AI makes code cheaper to *write* but exponentially more expensive to *read and maintain*.
* **Reddit (`r/ClaudeAI`, `r/programming`, `r/MachineLearning`):** Subreddits dedicated to specific tools and general programming are filled with daily friction reports. You will find mega-threads detailing how agentic tools often ignore user constraints, dive into infinite loops trying to fix their own broken tests, and burn through usage limits without producing a working solution.
* **AI Research & Benchmarking Communities:** Researchers working on benchmarks like **SWE-bench** (which tests an AI's ability to resolve real-world GitHub issues) frequently discuss how low the actual success rates are for complex, multi-file architectural changes. Researchers on X (formerly Twitter), such as François Chollet, constantly highlight the difference between an AI's memorization of code and actual software engineering intelligence.

---

### What the Community is Asking (and Realizing)

The conversation has absolutely shifted toward code quality, debugging, and simplification. The community is identifying several core issues that align exactly with your experience:

**1. The "Cognitive Debt" and "AI Slop" Crisis**
Developers are realizing that AI does not reduce maintenance costs; it shifts them. A common sentiment is that reviewing AI-generated code is inherently harder than reviewing human code because the AI is fluent and confident even when it is fundamentally wrong. The community is loudly questioning how to measure the "cognitive debt" created when an AI churns out brittle, complex spaghetti code that a human engineer eventually has to untangle.

**2. The Problem of "Solutionism" and Ignored Specs**
One of the most frequently discussed pitfalls of tools like Claude Code is their tendency to jump straight to implementation. Instead of acting as a junior developer who asks clarifying questions about a novel arithmetic spec, the AI will confidently steamroll ahead, making base-level assumptions just to output *something*. The community is actively begging for tools that pause, verify constraints, and engage in Test-Driven Development (TDD) rather than blindly generating code.

**3. The Illusion of Competence (Automation Bias)**
Security researchers and senior engineers are raising alarms about automation bias—the phenomenon where developers trust the AI's output because it looks syntactically perfect, missing the subtle, logical, or mathematical flaws underneath. Studies are showing that while AI speeds up raw output, it actively degrades the structural integrity of the codebase if not aggressively policed.

**4. The Pivot to Reduction and Verification**
The most advanced sectors of the community are asking exactly what you are: *How do we use this to make the codebase smaller, cleaner, and strictly verified?* There is a growing movement advocating that AI should be used primarily for writing tests, simplifying existing logic, and refactoring, rather than generating net-new implementations. However, because the models are heavily weighted to "produce" rather than "reduce," getting them to effectively simplify complex code requires fighting against their underlying design.

### Grounding the Reality for Novel Problems

If you are solving novel computer arithmetic problems, standard LLM chat interfaces and coding agents are currently the wrong tool for the actual heavy lifting. They cannot replace formal verification. For this specific niche, the academic and advanced engineering communities are shifting away from plain text generation and moving toward pairing LLMs with formal proof languages (like Lean or Coq) or aggressive property-based testing frameworks, forcing the AI to mathematically prove its output rather than just writing it.

Are you currently attempting to use any formal verification tools alongside these AI assistants, or are you relying entirely on the AI to interpret the text spec and write the logic directly?

## Separating concerns and context window management

We are relying on the AI to read and interpret the text spec, and these specs are large, dozens of pages. Furthermore, we have tried to clean-room OSS, so the spec is there in the form of a repo, and academic papers. We ask the AI to interpret the academic papers, synthesize an implementation plan, and refer to the repo as a possible oracle to know what good looks like. This has been interesting, not unproductive, but certainly not the amplifier we were hoping for. 

It makes complete sense that you are seeing a plateau in productivity with that workflow. What you are attempting—having an AI synthesize dozens of pages of academic papers, cross-reference an entire OSS repo as an oracle, and then synthesize a clean-room implementation of novel computer arithmetic—is exactly where the current architecture of LLMs fundamentally breaks down.

You aren't failing to use the tool correctly; you have found the hard edge of the tool's capabilities.

The advanced engineering and AI research communities are dissecting these exact failures right now. Here is where the conversations are happening and what the data is showing about why your specific workflow is stalling.

### The Research on "Formal Reasoning Failures"

The academic community has formalized what you are experiencing. A major 2026 survey published in TMLR on **"Large Language Model Reasoning Failures"** (and the highly active `Awesome-LLM-Reasoning-Failures` GitHub repository) categorizes these breakdowns extensively.

They classify what you are seeing as **Formal (Logical) Reasoning Failures** and **Compositional Breakdowns**.
When you ask an LLM to implement standard web CRUD logic, it uses informal, pattern-matching heuristic reasoning—which it is great at. When you ask it to synthesize novel computer arithmetic from an academic paper, it is forced to use formal symbolic logic. LLMs do not possess a true internal "scratchpad" for holding formal mathematical states over long contexts. If the next token isn't probabilistically obvious, the logical chain collapses, even if the spec explicitly tells it what to do.

### The Context Window Illusion

Your workflow relies heavily on the context window (papers + specs + the oracle repo). The engineering community has recently been analyzing the actual architecture of tools like Claude Code (for example, in the recent open-source analysis *Dive into Claude Code*), and the consensus is clear: **massive context windows are a trap for rigorous logic.**

While models technically accept 200K to 1M tokens, engineers are finding that **reasoning quality actively degrades once the context window exceeds 20% to 40% of its capacity.**

* When you load in a dozen-page math spec and an entire OSS repo, the model's attention mechanism gets spread too thin.
* It suffers from the "distractor effect." The model starts confusing the theoretical constraints in the academic paper with the implementation details it sees in the OSS repo.
* It loses the strict hierarchical discipline required for novel computer science. It will confidently blend a concept from page 3 of the paper with a structurally incompatible variable from the OSS repo.

### How the Community is Adapting for Clean-Room/Novel CS

Because directly feeding the whole payload into an agentic coder fails, the community pushing the boundaries on novel algorithms has shifted tactics. They are abandoning the "read this and build it" single-shot agent approach in favor of heavily constrained pipelines:

**1. Aggressive Context Isolation (The "Translate, Don't Synthesize" approach)**
Instead of giving one AI the paper, the spec, and the code, teams are separating the "understanding" from the "implementation."

* **Step 1:** Use an LLM solely to extract the math from the academic paper into a highly rigid pseudocode or formal specification language (like TLA+ or Lean).
* **Step 2:** Use a completely fresh, empty-context LLM session to translate that strict pseudocode into the target programming language. Never let the implementation model read the academic paper directly; the prose dilutes its coding focus.

**2. Test-Driven Development as a Hard Deterministic Gate**
Tools like Claude Code operate on a simple internal `while` loop. If you don't give them a deterministic way to fail, they will hallucinate success. Teams doing complex arithmetic are forcing the AI to write the test suite *first*, based entirely on the mathematical invariants in the paper. They run a "Stop Hook" that absolutely forbids the AI from writing the core logic until the human approves the test suite.

**3. Moving from AI Agents to Formal Provers**
For computer arithmetic specifically (where off-by-one bitwise errors or floating-point approximations are catastrophic), researchers are increasingly pairing LLMs with theorem provers (like Coq or Lean). They ask the LLM to write the proof of the arithmetic first, and only write the code once the proof compiles.

You are trying to use a probabilistic text generator to do deterministic mathematical engineering. The fact that it has been "interesting" rather than a total disaster is actually a testament to how far the models have come, but it will never be the clean-room amplifier you want until the workflow forces the AI out of its probabilistic comfort zone.

Have you tried breaking the pipeline down, perhaps having the AI just write the mathematical invariants or the property-based tests first, completely isolated from the implementation phase?
