import math
import matplotlib.pyplot as plt

q_min = 0 #(0,0,0,4,4,6,6,4,4,0,0,0)
q_max = 10 #(10,10,10,10,8,8,8,8,10,10,10,10)
q_end = 0
u_max = 4 #(4,4,2,2,1,1,1,1,2,2,4,4)
i_max = 4 #(4,4,2,2,1,1,1,1,2,2,4,4)
q_0 = 5
r = 0.04
T = 12
P_t = (20,22,25,18,15,15,20,19,21,12,22,25)
q_goal = 5
α = 0#.7

if type(q_min) == int:
    q_min = (q_min,)*T
if type(q_max) == int:
    q_max = (q_max,)*T
if type(u_max) == int:
    u_max = (u_max,)*T
if type(i_max) == int:
    i_max = (i_max,)*T
con = max(P_t)*max(q_max)
dis = [math.exp(-r*(t/T)) for t in range(1,T+1)]

def graph():
    graph = {(0,q_0): {},(T+1,q_end): {}}
    for i in range(T+1):
        x = [t for t in graph.keys() if t[0] == i]
        if i < T:
            for j in range(min(x)[1]-u_max[i],max(x)[1]+i_max[i]+1):
                if q_min[i] <= j <= q_max[i]:
                    graph[i+1,j] = {}
        for j in range(min(x)[1],max(x)[1]+1):
            if i < T:
                for k in range(j-u_max[i],j+i_max[i]+1):
                    if q_min[i] <= k <= q_max[i]: 
                       graph[i,j][i+1,k] = P_t[i]*(k-j)*dis[i]+con
            else:
                penalty = 0 if j == q_goal else α
                graph[i,j][i+1,q_end] = P_t[i-1]*(q_end-j)*dis[i-1]*(1-penalty)+con
    return(graph)

def Dijkstras_algorithm():
    full_graph = graph()
    length = {}
    for i in full_graph.keys():
        length[i] = math.inf
    length[0,q_0] = 0
    S = [i for i in full_graph.keys()]
    route = {}
    for i in full_graph.keys():
        route[i] = []
    while (T+1,q_end) in S:
        u = min(S, key=length.get)
        S.remove(u)
        if u[0] < T:
            for j in range(u[1]-u_max[u[0]],u[1]+i_max[u[0]]+1):
                if q_min[u[0]] <= j <= q_max[u[0]]:
                    alt_length = length[u]+full_graph[u][u[0]+1,j]
                    if alt_length < length[u[0]+1,j]:
                        length[u[0]+1,j] = alt_length
                        route[u[0]+1,j] = route[u] + [u]
        if u[0] == T:
            alt_length = length[u]+full_graph[u][T+1,q_end]
            if alt_length < length[T+1,q_end]:
                length[T+1,q_end] = alt_length
                route[T+1,q_end] = route[u] + [u]
    return(route[T+1,q_end],length[T+1,q_end])

def optimal_profit():
    optimal = Dijkstras_algorithm()
    optimal_route = optimal[0] + [(T+1,q_end)]
    optimal_length = round(-1*optimal[1]+(T+1)*con, 2)
    print(f'The optimal route goes through vertices {optimal_route} and makes a total profit of {optimal_length} euro.')
    plot_graph(optimal_route)
    
def plot_graph(route):
    full_graph = graph()
    for i in full_graph:
        plt.scatter(i[0], i[1], c='black')
        if i[0] < T:
            for j in range(i[1]-u_max[i[0]],i[1]+i_max[i[0]]+1):
                if q_min[i[0]] <= j <= q_max[i[0]]:
                    plt.arrow(i[0], i[1], 1, j-i[1], linewidth = 0.3, length_includes_head = True, head_width = 0.1, head_length = 0.25)
        if i[0] == T:
            plt.arrow(i[0], i[1], 1, q_end-i[1], linewidth = 0.3, length_includes_head = True, head_width = 0.1, head_length = 0.25)
    y = [x[1] for x in route]
    plt.xlabel("Time")
    plt.ylabel("Units of gas")
    plt.title("Optimization of profit from gas storage")
    plt.plot(y, 'red')
    
optimal_profit()