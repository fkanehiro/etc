from py_trees.composites import Sequence
from py_trees.common import Status
import itertools

##############################################################################
# Series (Not sure what a good name would be)
##############################################################################


class Series(Sequence):
    """
    Series are the factory lines of Behaviour Trees

    .. graphviz:: dot/sequence.dot

    A Series will progressively tick over each of its children so long as
    each child returns :data:`~py_trees.common.Status.SUCCESS`. If any child returns
    :data:`~py_trees.common.Status.FAILURE` or :data:`~py_trees.common.Status.RUNNING` the sequence will halt,
    all future children will be haulted and the parent will adopt
    the result of this child. If it reaches the last child, it returns with
    that result regardless.

    .. note::

       The Series halts once it sees a child is RUNNING and then returns
       the result. *It does not get stuck in the running behaviour*.

    .. seealso:: The :ref:`py-trees-demo-sequence-program` program demos a simple sequence in action.

    Args:
        name (:obj:`str`): the composite behaviour name
        children ([:class:`~py_trees.behaviour.Behaviour`]): list of children to add
        *args: variable length argument list
        **kwargs: arbitrary keyword arguments

    """
    def __init__(self, name="Series", children=None, *args, **kwargs):
        super(Series, self).__init__(name, children, *args, **kwargs)
        self.current_index = -1  # -1 indicates uninitialised

    def tick(self):
        """
        Tick over the children.

        Yields:
            :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children
        """
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        self.current_index = 0
        if self.status != Status.RUNNING:
            self.logger.debug("%s.tick() [resetting]" % self.__class__.__name__)
            # sequence specific handling
            for child in self.children:
                # reset the children, this helps when introspecting the tree
                if child.status != Status.INVALID:
                    child.stop(Status.INVALID)
            # subclass (user) handling
            self.initialise()
        # run any work designated by a customised instance of this class
        self.update()
        for child in self.children:
            for node in child.tick():
                yield node
                if node is child and node.status != Status.SUCCESS:
                    self.status = node.status
                    # If status is running all future nodes must be stoped in case they are already running
                    for child in itertools.islice(self.children, self.current_index + 1, None):
                        if child.status != Status.INVALID:
                            child.stop(Status.INVALID)
                    yield self
                    return
            self.current_index += 1
        # At this point, all children are happy with their SUCCESS, so we should be happy too
        self.current_index -= 1  # went off the end of the list if we got to here
        self.stop(Status.SUCCESS)
        yield self
