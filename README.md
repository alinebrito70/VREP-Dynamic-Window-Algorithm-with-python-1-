Dynamic Window Approach (DWA) no CoppeliaSim

Projeto desenvolvido para a disciplina de Robótica I utilizando o algoritmo Dynamic Window Approach (DWA) para navegação autônoma de um robô móvel no ambiente de simulação CoppeliaSim.

Objetivo

Implementar um sistema capaz de:

seguir um alvo (Target)
evitar obstáculos
navegar autonomamente
utilizar sensores de proximidade
controlar movimento diferencial das rodas

Toda a lógica foi desenvolvida em Python utilizando comunicação com o CoppeliaSim via ZMQ Remote API.

Tecnologias Utilizadas

Python 3
CoppeliaSim Edu
ZMQ Remote API
VS Code
GitHub
Funcionalidades
Navegação autônoma
Desvio de obstáculos
Sistema de fuga para regiões presas
Controle diferencial das rodas
Parada automática ao alcançar o objetivo

Estrutura do Projeto
📂 VREP-Dynamic-Window-Algorithm-with-python

│

├── dwa_create2_zmq.py

├── teste_conexao_zmq.py

├── script_create.py

├── script_mirror.py

├── cenario_dwa_funcional.ttt

├── Relatorio_Final_Robotica.pdf

└── README.md

Como Executar
1. Abrir o cenário no CoppeliaSim
cenario_dwa_funcional.ttt
2. Iniciar a simulação
Simulation -> Start
3. Executar o código Python
python dwa_create2_zmq.py
Resultados

O robô consegue:

localizar o Target
evitar obstáculos
sair de regiões presas
navegar autonomamente no ambiente

Referências
FOX, Dieter; BURGARD, Wolfram; THRUN, Sebastian. The Dynamic Window Approach to Collision Avoidance.
PythonRobotics — Dynamic Window Approach
Documentação oficial do CoppeliaSim
Documentação oficial da ZMQ Remote API
