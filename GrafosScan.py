import heapq
from collections import deque
import networkx as nx
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import messagebox, scrolledtext

class Grafo:
    def __init__(self, numero_vertices=0):
        self.vertices = numero_vertices
        self.matriz_adjacencia = [[None] * numero_vertices for _ in range(numero_vertices)]
    
    def carregar_grafo_de_arquivo(self, arquivo):
        """Carrega o grafo a partir de um arquivo."""
        with open(arquivo, 'r') as f:
            self.vertices = int(f.readline().strip())
            self.matriz_adjacencia = [[None] * self.vertices for _ in range(self.vertices)]
            for linha in f:
                v1, v2, peso = linha.strip().split()
                self.adicionar_aresta(int(v1), int(v2), float(peso))
    
    def adicionar_aresta(self, v1, v2, peso=1):
        self.matriz_adjacencia[v1 - 1][v2 - 1] = peso
        self.matriz_adjacencia[v2 - 1][v1 - 1] = peso
    
    def ordem(self):
        return self.vertices

    def tamanho(self):
        return sum(1 for i in range(self.vertices) for j in range(i + 1, self.vertices) if self.matriz_adjacencia[i][j] is not None)

    def densidade(self):
        if self.vertices < 2:
            return 0
        return (2 * self.tamanho()) / (self.vertices * (self.vertices - 1))

    def vizinhos(self, vertice):
        return [i + 1 for i in range(self.vertices) if self.matriz_adjacencia[vertice - 1][i] is not None]

    def grau(self, vertice):
        return len(self.vizinhos(vertice))

    def possui_ciclo(self):
        def dfs(v, visitado, parent):
            visitado[v] = True
            for vizinho in self.vizinhos(v + 1):
                vizinho -= 1
                if not visitado[vizinho]:
                    if dfs(vizinho, visitado, v):
                        return True
                elif vizinho != parent:
                    return True
            return False

        visitado = [False] * self.vertices
        for i in range(self.vertices):
            if not visitado[i]:
                if dfs(i, visitado, -1):
                    return True
        return False

    def caminho_minimo(self, vertice):
    # Inicializa todas as distâncias como infinito
     dist = [float('inf')] * self.vertices
     dist[vertice - 1] = 0  # Distância do vértice de origem para ele mesmo é 0

    # Relaxa as arestas repetidamente
     for _ in range(self.vertices - 1):
        for u in range(self.vertices):
            for v in range(self.vertices):
                peso = self.matriz_adjacencia[u][v]
                
                # Converte o peso para positivo se ele for negativo
                if peso is not None:
                    peso = abs(peso)
                    
                # Relaxamento da aresta se houver uma conexão válida
                if peso is not None and dist[u] + peso < dist[v]:
                    dist[v] = dist[u] + peso

    # Verificação de ciclos negativos (não é necessário aqui, mas mantido para compatibilidade)
     for u in range(self.vertices):
        for v in range(self.vertices):
            peso = self.matriz_adjacencia[u][v]
            if peso is not None:
                peso = abs(peso)  # Converte para positivo
                if dist[u] + peso < dist[v]:
                    return "O grafo contém um ciclo negativo, impossível calcular caminho mínimo."

     return dist




    def eh_articulacao(self, vertice):
        visitado = [False] * self.vertices
        disc = [float('inf')] * self.vertices
        low = [float('inf')] * self.vertices
        parent = [None] * self.vertices
        pontos_articulacao = set()

        def dfs(v, tempo):
            visitado[v] = True
            disc[v] = low[v] = tempo
            filhos = 0
            for u in self.vizinhos(v + 1):
                u -= 1
                if not visitado[u]:
                    parent[u] = v
                    filhos += 1
                    dfs(u, tempo + 1)
                    low[v] = min(low[v], low[u])
                    if parent[v] is None and filhos > 1:
                        pontos_articulacao.add(v)
                    elif parent[v] is not None and low[u] >= disc[v]:
                        pontos_articulacao.add(v)
                elif u != parent[v]:
                    low[v] = min(low[v], disc[u])

        dfs(vertice - 1, 0)
        return (vertice - 1) in pontos_articulacao

    def componentes_conexas(self):
        visitado = [False] * self.vertices
        componentes = []

        def dfs(v, componente):
            visitado[v] = True
            componente.append(v + 1)
            for vizinho in self.vizinhos(v + 1):
                if not visitado[vizinho - 1]:
                    dfs(vizinho - 1, componente)

        for i in range(self.vertices):
            if not visitado[i]:
                componente = []
                dfs(i, componente)
                componentes.append(componente)

        return len(componentes), componentes

    def busca_largura_com_arestas_nao_arvore(self, vertice):
        """Executa a BFS e identifica as arestas não-árvore."""
        visitado = [False] * self.vertices
        fila = deque([vertice - 1])
        visitado[vertice - 1] = True
        sequencia_visitada = []
        arestas_nao_arvore = []
        arvore_arestas = set()  # Para armazenar as arestas da árvore

        while fila:
            u = fila.popleft()
            sequencia_visitada.append(u + 1)
            for v in self.vizinhos(u + 1):
                v -= 1
                if not visitado[v]:
                    visitado[v] = True
                    fila.append(v)
                    arvore_arestas.add((u, v))
                elif (u, v) not in arvore_arestas and (v, u) not in arvore_arestas:
                    arestas_nao_arvore.append((u + 1, v + 1))

        return sequencia_visitada, arestas_nao_arvore

    def gerar_networkx_grafo(self):
        G = nx.Graph()
        for v1 in range(self.vertices):
            for v2 in range(v1 + 1, self.vertices):
                peso = self.matriz_adjacencia[v1][v2]
                if peso is not None:
                    G.add_edge(v1 + 1, v2 + 1, weight=peso)
        return G

    def desenhar_grafo(self):
        G = self.gerar_networkx_grafo()
        pos = nx.spring_layout(G)
        plt.figure(figsize=(8, 6))
        nx.draw_networkx_nodes(G, pos, node_size=700, node_color="skyblue")
        nx.draw_networkx_edges(G, pos, width=2, edge_color="gray")
        nx.draw_networkx_labels(G, pos, font_size=12, font_color="black", font_family="sans-serif")
        edge_labels = nx.get_edge_attributes(G, 'weight')
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
        plt.title("Visualização do Grafo")
        plt.axis("off")
        plt.show()

class GrafoApp:
    def __init__(self, root, grafo):
        self.grafo = grafo
        self.root = root
        self.root.title("Aplicação de Grafos")
        self.root.geometry("600x600")
        
        title = tk.Label(root, text="Menu de Operações com Grafos", font=("Helvetica", 16, "bold"), bg="#4B9CD3", fg="white")
        title.pack(fill="x")

        frame_entrada = tk.Frame(root, bg="#E1E1E1", padx=10, pady=10)
        frame_entrada.pack(pady=5, fill="x")
        self.label_vertice = tk.Label(frame_entrada, text="Digite o vértice:", bg="#E1E1E1")
        self.label_vertice.grid(row=0, column=0, sticky="w")
        self.entry_vertice = tk.Entry(frame_entrada)
        self.entry_vertice.grid(row=0, column=1)

        self.frame_opcoes = tk.Frame(root, bg="#D9D9D9", padx=10, pady=10)
        self.frame_opcoes.pack(pady=5, fill="x")
        self.criar_botoes()

        frame_resultado = tk.Frame(root, bg="#FFFFFF", padx=10, pady=10)
        frame_resultado.pack(pady=5, fill="both", expand=True)
        self.resultado = scrolledtext.ScrolledText(frame_resultado, width=50, height=15, bg="#F0F0F0", font=("Arial", 12))
        self.resultado.pack(fill="both", expand=True)

        btn_limpar = tk.Button(root, text="Limpar Resultados", command=self.limpar_resultados, bg="#FF6347", fg="white")
        btn_limpar.pack(pady=5)

    def criar_botoes(self):
        botoes = [
            ("Ordem do Grafo", self.exibir_ordem),
            ("Tamanho do Grafo", self.exibir_tamanho),
            ("Densidade do Grafo", self.calcular_densidade),
            ("Vizinhos de um Vértice", self.exibir_vizinhos),
            ("Grau de um Vértice", self.exibir_grau),
            ("Verificar Ciclo", self.verificar_ciclo),
            ("Caminho Mínimo", self.calcular_caminho_minimo),
            ("Verificar Articulação", self.verificar_articulacao),
            ("Componentes Conexas", self.detectar_componentes_conexas),
            ("Visualizar Grafo", self.visualizar_grafo),
            ("BFS e Arestas Não-Árvore", self.executar_bfs)
        ]
        
        for i, (texto, comando) in enumerate(botoes):
            botao = tk.Button(self.frame_opcoes, text=texto, command=comando, width=25, bg="#4B9CD3", fg="white")
            botao.grid(row=i // 2, column=i % 2, padx=5, pady=5, sticky="ew")

    def exibir_ordem(self):
        ordem = self.grafo.ordem()
        self.resultado.insert(tk.END, f"Ordem do Grafo: {ordem}\n")

    def exibir_tamanho(self):
        tamanho = self.grafo.tamanho()
        self.resultado.insert(tk.END, f"Tamanho do Grafo: {tamanho}\n")

    def calcular_densidade(self):
        densidade = self.grafo.densidade()
        self.resultado.insert(tk.END, f"Densidade do Grafo: {densidade}\n")

    def exibir_vizinhos(self):
        vertice = self._obter_vertice()
        if vertice is not None:
            vizinhos = self.grafo.vizinhos(vertice)
            self.resultado.insert(tk.END, f"Vizinhos do Vértice {vertice}: {vizinhos}\n")

    def exibir_grau(self):
        vertice = self._obter_vertice()
        if vertice is not None:
            grau = self.grafo.grau(vertice)
            self.resultado.insert(tk.END, f"Grau do Vértice {vertice}: {grau}\n")

    def verificar_ciclo(self):
        possui_ciclo = self.grafo.possui_ciclo()
        msg = "Sim" if possui_ciclo else "Não"
        self.resultado.insert(tk.END, f"O grafo possui ciclo? {msg}\n")

    def calcular_caminho_minimo(self):
     vertice = self._obter_vertice()
     if vertice is not None:
        distancias = self.grafo.caminho_minimo(vertice)
        if isinstance(distancias, str):
            self.resultado.insert(tk.END, distancias + "\n")
        else:
            self.resultado.insert(tk.END, f"Caminho Mínimo a partir do Vértice {vertice}:\n")
            for i, d in enumerate(distancias, start=1):
                # Formata a distância com duas casas decimais
                distancia_formatada = f"{d:.2f}" if d != float('inf') else "inf"
                self.resultado.insert(tk.END, f"Vértice {i}: {distancia_formatada}\n")


    def verificar_articulacao(self):
        vertice = self._obter_vertice()
        if vertice is not None:
            eh_articulacao = self.grafo.eh_articulacao(vertice)
            msg = "é" if eh_articulacao else "não é"
            self.resultado.insert(tk.END, f"O Vértice {vertice} {msg} uma articulação.\n")

    def detectar_componentes_conexas(self):
        num_componentes, componentes = self.grafo.componentes_conexas()
        self.resultado.insert(tk.END, f"Número de Componentes Conexas: {num_componentes}\n")
        for i, componente in enumerate(componentes, start=1):
            self.resultado.insert(tk.END, f"Componente {i}: {componente}\n")

    def executar_bfs(self):
        vertice = self._obter_vertice()
        if vertice is not None:
            sequencia, arestas_nao_arvore = self.grafo.busca_largura_com_arestas_nao_arvore(vertice)
            self.resultado.insert(tk.END, f"Sequência de vértices visitados na BFS a partir do vértice {vertice}: {sequencia}\n")
            self.resultado.insert(tk.END, f"Arestas que não fazem parte da árvore de busca: {arestas_nao_arvore}\n")

    def visualizar_grafo(self):
        self.grafo.desenhar_grafo()

    def limpar_resultados(self):
        self.resultado.delete("1.0", tk.END)

    def _obter_vertice(self):
        try:
            vertice = int(self.entry_vertice.get())
            if vertice < 1 or vertice > self.grafo.ordem():
                raise ValueError("O vértice selecionado não existe no grafo.")
            return vertice
        except ValueError as e:
            messagebox.showerror("Erro", str(e))
            return None

# Inicialização do grafo e interface gráfica
if __name__ == "__main__":
    grafo = Grafo()
    grafo.carregar_grafo_de_arquivo('grafo.txt')
    root = tk.Tk()
    app = GrafoApp(root, grafo)
    root.mainloop()
