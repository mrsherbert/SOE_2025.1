# SOE_2025.1
Segmentação de faixa para sistema LKA
Link pra overleaf https://www.overleaf.com/4656859685hqrrjnyxwysz#b49b99


Para o PC1.

O tema escolhido foi Segmentação de faixa para controle LCA embarcado

O problema a ser resolvido é que atualmente no veículo de pequena escala do projeto Segurauto, se utiliza de um computador especializado (ZED) da Stereo Labs para se fazer a detecção de faixa e para se calcular o ângulo de roda necessário para se fazer uma curva. Enquanto isto funciona adequadamente e proporciona facilidade na atualização do modelo, o consumo energético do sistema está sendo um problema recorrente, já que a Zed Box necessita de 30W 12v, e como ela não possuí regulador de tensão interno é necessário utilizar da fonte externa de 220v do produto, a qual demanda o uso de um inversor 12v para 220v~ já que a alimentação do veículo vem de uma única bateria de 12v. [Em média indo da batéria 100% da pra deixar ligado por 6 horas antes dela acabar mas ainda vai ser colocado mais componentes no veículo então o consumo vai ficar pior.]

Também o custo da Zed Box faz com que a mesma seja menos desejada para uma implementação final do sistema, dando-se a necessidade da procura de alternativas para a implementação do sistema.

Para que o projeto seja concluido satisfatoriamente é necessário que o modelo implementado na raspberry seja capaz de detectar as faixas do circuito em imagens e enviar ações de controle com velocidade o suficiente para que o veículo se mantenha no circuito a uma velocidade média selecionada com o controle de cruzeiro já implementado (O jeito que ele funciona é jogar um valor de 0-115 e ele fica nessa velocidade fixa, depois eu transformo isso em uma medida física real).

É necessário o método de segmentação para detecção de faixa já que o método clássico de comparar cores/iluminação é extremamente limitado pelo estado da estrada, para fazer com que o modelo clássico para de funcionar basta trocar a iluminação da estrada (que pode ser feito ao se testar em diferentes horas do dia), e sim é possível fazer ele funcionar perfeitamente pra diversos cenários, mas até onde eu sei é extremamente difícil e requer uma equipe num projeto especificamente pra fazer funcionar.

Um benefício extra é que se funcionar na rasperry da pra mostrar que é comerciavel, outra coisa que não coloquei aqui é que estou tentando embarcar o mesmo sistema em FPGA, então mesmo que não sejamos capazes de fazer rodar bem na rasperry apenas ter o tempo de processamento médio de cada comando pode-se dizer que deu certo ' ')b .

Eu vou ir pegar as referências com o pessoal que foca mais na área de visão computacional do Segurauto que eu normalmente lido com outros sistemas.


A fazer-

Implementar NCNN na raspberry pi 3

Implementar modulo MCP-CAN

Implementar no veículo de pequena escala

Documento PC2

Documento PC1-feito 

Adquirir melhores imagens do veículo

Adquirir estrutura do software-feito

Adquirir o modelo-feito

Adquirir a teoria por trás do modelo-feito

Adquirir o método de treinamento do modelo pra citar no documento

Adquirir vídeo do sistema funcionando pra colocar no documento

Adquirir artigos sobre (vou ver se ja tem uma lista)
