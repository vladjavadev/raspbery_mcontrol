class PathObservation:
    is_updated = False
    next_pos = None
    is_done = False

    def update(self, new_next_pos):
        self.next_pos = new_next_pos
        self.is_updated = True