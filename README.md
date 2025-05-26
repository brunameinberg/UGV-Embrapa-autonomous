# UGV-Embrapa-autonomous

## O que é este projeto?  
Este projeto tem como objetivo desenvolver um Veículo Terrestre Não Tripulado (UGV) para monitoramento de áreas frutíferas e florestais, inspirado no projeto Open Source Rover da NASA. O UGV foi projetado para navegar de forma autônoma e segura entre fileiras de plantações, fornecendo dados valiosos para pesquisa e manejo agrícola.

## Contexto  
Este UGV foi desenvolvido por alunos do Insper como parte do seu Projeto de Conclusão de Curso (Capstone) em Engenharia, em colaboração com a Embrapa (Empresa Brasileira de Pesquisa Agropecuária), durante o primeiro e segundo semestres de 2024. O trabalho aproveita a base do projeto Open Source Rover do JPL/NASA, adaptando-o para aplicações agrícolas.

## Contribuidores
- Breno Alencar Araújo
- Bruno Morales Balkins
- Fernando Bichuette Assumpção
- Giulia Carolina Martins de Sampaio
- Felipe Catapano Emrich Melo
- Rafael Eli Katri
- Gabriel Brunoro Motta Tumang
- Luana de Matos Sorpreso
- Bruna Lima Meinberg
- Guilherme Garrido Klingelfus Pinheiro
- Matheus Ribeiro Barros
- Rafael Pacheco Paolino
  
- Orientador: Prof. Dr. Vinícius Licks
- Mentor: Thiago Teixeira Santos

## Estrutura
```plaintext
UGV-Embrapa-autonomous/
├── .gitignore
├── ugv_embrapa_ws/
    └── install/
    └── log/
    └── src/
        └── osr_control/
        └── osr_bringup/
        └── osr_gazebo/
        └── osr_interfaces/
        └── osr_autonomous/
├── README.md
```

## Código Relacionado ao ROS
 
- **osr_control**: código central que se comunica com os drivers de motor e escuta comandos.  
- **osr_interfaces**: definições de mensagens customizadas.  
- **osr_bringup**: arquivos de configuração e launch para inicializar o rover.  


---

### osr_control  
- **roboclaw.py**: cópia da biblioteca Python Roboclaw, API para os controladores Roboclaw. Agnóstico ao ROS.  
- **roboclaw_wrapper.py**: nó ROS que encapsula e abstrai a biblioteca Roboclaw. Recebe comandos e reporta o estado de cada motor.  
- **servo_control.py**: nó ROS que recebe comandos do nó principal do rover para posicionar os servos de canto em determinado ângulo e repassa esses comandos ao chip PCA9685.  
- **rover.py**: nó ROS que controla o rover, recebendo comandos de alto nível, calculando os comandos de motor e enviando-os para o `roboclaw_wrapper.py`.  

---

### osr_interfaces  
Contém as definições de mensagens customizadas utilizadas pelo rover. Consulte cada definição de mensagem para detalhes e unidades.

---

### osr_bringup  
O pacote **osr_bringup** inclui o arquivo de launch necessário para iniciar todos os nós ROS, bem como os parâmetros de operação do robô.

---

### osr_gazebo  
Os seguintes pacotes ROS estão incluídos para visualizar o rover no RViz e simular suas operações no Gazebo:

- **rviz.launch**: inicia o ambiente de visualização no RViz, permitindo acompanhar em tempo real os movimentos do rover e os dados dos sensores.  
- **empty_world.launch**: carrega o modelo do rover em um mundo vazio no Gazebo, criando um campo de testes virtual para simular suas operações.  

---

### osr_autonomous  
O pacote **osr_autonomous** implementa a lógica e algoritmos para navegação autônoma do rover:

