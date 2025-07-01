This script solves the VRP with time windows and capacity constraints using MILP formulation.
Since the in the provided problem, we do not differentiate between vehicles, we can reduce the 
variables if the problem by not iterating over vehicles. But care shoud be take to ensure different
tours and avoiding subtours, at the same time adhereing to the given Time window and Capacity constraints. 

Author: Srinivasragavan Venkatasubramanian


## Model Formulation


## Objective 
>>>>### Minimize
>>>>### $`\displaystyle\sum\limits_{\substack{i \in \mathcal{N} \\ j \in \mathcal{N} \\ i \ne j}} \mathcal{Dist}_{i,j}  x_{i,j}`$

## S.T
>### Exit route arc constraints
>>>>#### $\displaystyle\sum\limits_{\substack{j \in \mathcal{N} \\ j \ne i}} x_{i, j} = 1 \hspace{12cm} \forall i \in \mathfrak{N}$
>>>>#### $\displaystyle\sum\limits_{\substack{j \in \mathfrak{N} }} x_{DEPOT, j} \le |\mathcal{V}|$
>### Enit route arc constraints
>>>>#### $\displaystyle\sum\limits_{\substack{i \in \mathcal{N} \\ j \ne i}} x_{i, j} = 1 \hspace{12cm} \forall j \in \mathfrak{N}$
>>>>#### $\displaystyle\sum\limits_{\substack{j \in \mathfrak{N} }} x_{j, DEPOT} \le |\mathcal{V}|$
>### Node Consistency
>>>>#### $\displaystyle\sum\limits_{\substack{i \in \mathcal{N} \\ h \ne i}} x_{i, h} - \displaystyle\sum\limits_{\substack{j \in \mathcal{N} \\ h \ne j}} x_{h, j} = 0 \hspace{10cm} \forall h \in \mathcal{N}$
>### Demand Continuity
>>>>#### $`c_{i, j} \leq \mathcal{Q} * x_{i, j} \hspace{14cm} \forall i,j \in \mathcal{N}; i \ne j`$
>>>>#### $`c_{i, j} \geq \mathcal{D}_{i} - (1 - x_{i, j}) * \mathcal{Q} \hspace{10cm}  \forall i,j \in \mathcal{N}; i \ne j`$
>>>>#### $`\displaystyle\sum\limits_{\substack{j \in \mathcal{N} \\ j \ne i}} c_{i, j} - \displaystyle\sum\limits_{\substack{j \in \mathcal{N} \\ j \ne i}} c_{j, i} = \mathfrak{D}_{i} \hspace{12.8cm} \forall i \in \mathfrak{N}`$
>### Time Constraints
>>>>#### $`u_{i} \geq \mathcal{Service\_Window\_Start}_{i} \hspace{11.5cm} \forall i \in \mathfrak{N}`$
>>>>#### $`u_{i} \leq \mathcal{ServiceWindowStart}_{i} + \mathcal{ServiceWindowDuration}_{i} - \mathcal{ServiceTime}_{i} \hspace{3.8cm} \forall i \in \mathfrak{N}`$
>### Time Continuity
>>>>#### $`u_{i} + \mathcal{ServiceTime}_{i} + \mathcal{TTime}_{i, j} - u_{j} \leq  (1 - x_{i, j}) * BigM \hspace{7.6cm} \forall i,j \in \mathfrak{N}; i \ne j`$
>### (otional) if no wait times allowed during tour
>>>>#### $`u_{i} + \mathcal{ServiceTime}_{i} + \mathcal{TTime}_{i, j} - u_{j} \geq  (x_{i, j} - 1) * BigM \hspace{7.6cm} \forall i,j \in \mathfrak{N}; i \ne j`$
>### (optional )f wait allowed only during service window
>>>>#### $`d_{i} \geq u_{i} + \mathcal{ServiceTime}_{i}  \hspace{12.5cm} \forall i \in \mathfrak{N}`$
>>>>#### $`d_{i} \leq u_{i} + \mathcal{ServiceTime}_{i} + \mathcal{ServiceDuration}_{i} \hspace{9.3cm} \forall i \in \mathfrak{N}`$
>>>>#### $`d_{i} + \mathcal{TTime}_{i, j} \leq u_{j} + (1 - x_{i, j}) * BigM \hspace{10.2cm} \forall i,j \in \mathfrak{N}; i \ne j`$
>>>>#### $`d_{i} + \mathcal{TTime}_{i, j} \geq u_{j} - (1 - x_{i, j}) * BigM \hspace{10.2cm} \forall i,j \in \mathfrak{N}; i \ne j`$
>### Tour start time variable constraints
>>>>#### $`t_{i} \leq u_{i} - \mathcal{TTime}_{i, DEPOT} + (1 - x_{i, j}) * BigM \hspace{9.6cm} \forall i \in \mathfrak{N}`$
>>>>#### $`t_{i} \geq u_{i} - \mathcal{TTime}_{i, DEPOT} - (1 - x_{i, j}) * BigM \hspace{9.6cm} \forall i \in \mathfrak{N}`$
>>>>#### $`t_{j} - t_{i} \leq (1 - x_{i, j}) * BigM \hspace{12.2cm} \forall i, j \in \mathfrak{N}; i \ne j`$
>>>>#### $`t_{i} - t_{j} \leq (1 - x_{i, j}) * BigM \hspace{12.2cm} \forall i, j \in \mathfrak{N}; i \ne j`$
>### Time limit constraint
>>>>#### $`u_{i} + \mathcal{ServiceTime}_{i} + \mathcal{TTime}_{i, DEPOT} - t_{i} \leq \mathcal{T} \hspace{9.4cm} \forall i \in \mathfrak{N}`$

>>>>>>>>>>>>>#### $x_{i, j}$ binary -> route arc decision variable
>>>>>>>>>>>>>#### $u_{i} \in \mathbb{R}^{+}$ -> service start time at node i
>>>>>>>>>>>>>#### $d_{i} \in \mathbb{R}^{+}$ -> departure time from node i
>>>>>>>>>>>>>#### $c_{i, j} \in \mathbb{R}^{+}$ -> truck load between node i and j
>>>>>>>>>>>>>#### $t_{i} \in \mathbb{R}^{+}$ -> tour start time; same val for all nodes in a tour
>>>>>>>>>>>>>#### $\mathcal{N}$ -> set of all nodes including DEPOT
>>>>>>>>>>>>>#### $\mathfrak{N}$ -> set of all nodes except DEPOT
>>>>>>>>>>>>>#### $\mathcal{V}$ -> set of all vehicles
>>>>>>>#### Parameters
>>>>>>>>>>>>>#### $\mathcal{Q}$ -> Capacity Limit of a Vehicle
>>>>>>>>>>>>>#### $\mathcal{D}_{i}$ -> Demand at Node i
>>>>>>>>>>>>>#### $\mathcal{T}$ -> Time Limit for each tour
>>>>>>>>>>>>>#### $\mathcal{Dist}_{i, j}$ -> Distance Matrix between nodes i and j
>>>>>>>>>>>>>#### $\mathcal{TTime}_{i, j}$ -> Trave Time matrix between nodes i and j
