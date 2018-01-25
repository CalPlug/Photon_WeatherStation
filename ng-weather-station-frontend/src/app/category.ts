export class Category{
    name: string;
    children: string[] = [];
    constructor(name: string) {
        this.name = name;
    }
}