# Método da Janela Dinâmica (DWA) no CoppeliaSim

Projeto desenvolvido para a disciplina de Robótica I utilizando o método da Janela Dinâmica (Dynamic Window Approach - DWA) aplicado ao robô móvel do laboratório no ambiente de simulação CoppeliaSim.

---

## Objetivo

Implementar o algoritmo Dynamic Window Approach (DWA) para navegação autônoma, desvio de obstáculos e planejamento local de trajetória utilizando Python integrado ao CoppeliaSim via ZMQ Remote API.

O principal objetivo foi permitir que o robô se deslocasse até uma posição alvo (Target), evitando colisões com obstáculos e respeitando limites de segurança durante a navegação.

---

## Fundamentação Teórica

O método da Janela Dinâmica (DWA), proposto por Dieter Fox, Wolfram Burgard e Sebastian Thrun em 1997, é um algoritmo de navegação local para robôs móveis que seleciona velocidades lineares e angulares seguras em tempo real.

O algoritmo avalia três critérios principais:

* **Heading** → direção até o objetivo
* **Dist** → distância segura até obstáculos
* **Velocity** → preferência por velocidades mais eficientes

A partir disso, o robô escolhe a melhor combinação de velocidade linear (v) e velocidade angular (w), prevendo trajetórias futuras antes de executar o movimento.

---

## Tecnologias Utilizadas

* Python
* NumPy
* CoppeliaSim Edu
* ZMQ Remote API
* VS Code
* GitHub

---

## Funcionalidades Implementadas

* Controle do robô pelo VS Code
* Integração Python + CoppeliaSim
* Leitura dos sensores de proximidade
* Navegação até o Target
* Sistema de emergência para obstáculos próximos
* Controle diferencial das rodas
* Parada segura antes de alcançar o objetivo

---

## Estrutura do Projeto

* `dwa_create2_zmq.py` → implementação principal do DWA
* `teste_conexao_zmq.py` → teste de conexão com o CoppeliaSim
* `BASE_FUNCIONAL_INVISIVEL (certo).ttt` → cena principal de simulação
* `script_create.py` → scripts auxiliares
* `script_mirror.py` → scripts auxiliares
* `Relatorio.pdf` → relatório final do projeto

---

## Como Executar

### 1. Abrir a cena no CoppeliaSim

Abrir o arquivo:

```bash
BASE_FUNCIONAL_INVISIVEL (certo).ttt
```

---

### 2. Verificar os objetos da cena

Garantir que:

* os sensores estejam funcionando corretamente
* os obstáculos estejam configurados como **Detectable**
* o objeto **Target** esteja posicionado corretamente

---

### 3. Ativar a comunicação via ZMQ Remote API

Manter a simulação pronta para receber o controle externo em Python.

---

### 4. Executar no terminal

```bash
python dwa_create2_zmq.py
```

---

## Resultado

O robô consegue navegar autonomamente até o Target, desviando de obstáculos e evitando colisões, utilizando o método da Janela Dinâmica para tomada de decisão em tempo real.

O sistema também realiza parada segura antes de alcançar o objetivo, respeitando a margem de tolerância definida no algoritmo.

---

video demonstrativo na pasta, chamado de "Video do robo duncionando.mp4"

## Referências

FOX, Dieter; BURGARD, Wolfram; THRUN, Sebastian. *The Dynamic Window Approach to Collision Avoidance.*

PythonRobotics – Dynamic Window Approach.

Material da disciplina – JanelaDinamica.pdf.

Documentação oficial do CoppeliaSim.

Documentação oficial da ZMQ Remote API.

