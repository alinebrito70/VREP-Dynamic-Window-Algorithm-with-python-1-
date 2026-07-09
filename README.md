# Navegação Autônoma com A* Aprimorado e DWA

Projeto desenvolvido para a disciplina de **Robótica I** da **UNIVASF**, com o objetivo de implementar uma arquitetura híbrida de navegação autônoma para um robô móvel diferencial no **CoppeliaSim**, utilizando **Python** e a **ZeroMQ Remote API**.

O sistema combina:

- **A\* Aprimorado** para planejamento global da rota;
- **Grade de ocupação** para representar o ambiente e os obstáculos;
- **DWA (Dynamic Window Approach)** para controle local e desvio de obstáculos;
- **CoppeliaSim** para simulação do robô e do ambiente.

---

## Funcionamento

O fluxo principal do projeto é:

1. Conectar o Python ao CoppeliaSim;
2. Capturar o ambiente com um sensor de visão;
3. Gerar a grade de ocupação;
4. Calcular a rota global com A\*;
5. Selecionar pontos-chave da rota;
6. Usar o DWA para gerar comandos de velocidade;
7. Enviar os comandos para o robô;
8. Navegar até o objetivo evitando obstáculos.

---

## Estrutura do Projeto
```text
.
├── main.py
├── dynamic_window_approach.py
├── mapa_ocupacao.py
├── utils/
│   ├── testar_conexao.py
│   └── listar_objetos.py
├── cenas/
│   └── cena_com_obstaculos.ttt
├── docs/
│   ├── Relatorio_Final_Navegacao_Autonoma.pdf
│   ├── Apresentacao_Navegacao_Autonoma.pptx
│   └── figuras/
└── legado/
```


## Pré-requisitos
Python 3.8 ou superior;
CoppeliaSim;
ZeroMQ Remote API.

---

## Instale as dependências principais:

pip install numpy coppeliasim-zmqremoteapi-client

Dependências opcionais para geração de figuras e documentos:

pip install matplotlib reportlab python-pptx pillow

---

## Como Executar
Abra o CoppeliaSim.
Carregue a cena:
cenas/cena_com_obstaculos.ttt , 
Teste a conexão, se necessário:
python utils/testar_conexao.py. 
Execute o projeto:
python main.py


