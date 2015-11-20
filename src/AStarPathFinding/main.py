import pygame
from AStar import AStar
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
scale = 2
SIZE = (200*scale,200*scale)
WIDTH = 10*scale
HEIGHT = 10*scale

def main():
    pygame.init()
    screen = pygame.display.set_mode(SIZE)
    pygame.display.set_caption("A* Path Finding")
    done = False
    clock = pygame.time.Clock()
    #Run AStar path finding with start of 1,1 and goal of 20,20
    star = AStar((0,0),(19,19))
    star.run()
    for node in star.path:
        print 'Coordinates ({},{}), g_score: {}'.format(node.index[0]+1, node.index[1]+1, node.g_score) 
    while not done:
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                done = True  # Flag that we are done so we exit this loop
        screen.fill(WHITE)
        drawing = True
        world = star.world.tolist()
        for y in range(0, len(world)):
            for x in range(0, len(world[y])):
                if world[y][x] == 1:
                    pygame.draw.rect(screen, BLACK, [WIDTH*x, HEIGHT*y, WIDTH, HEIGHT])
        for node in star.path:
            if (star.path.index(node)+1)<len(star.path):
                nextNode = star.path[(star.path.index(node)+1)]
                start = [node.index[0]*WIDTH, node.index[1]*HEIGHT]
                end = [nextNode.index[0]*WIDTH, nextNode.index[1]*HEIGHT]
                pygame.draw.line(screen, BLUE, start, end, WIDTH)
                
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == '__main__':
    main()



